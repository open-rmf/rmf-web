import * as ChildProcess from 'child_process';

enum LaunchMode {
  None,
  Local,
  LocalSingleton,
}

interface Launcher {
  launch(): Promise<void>;
  kill(): Promise<void>;
}

/**
 * Creates a launcher based on the launch mode. If `launchMode` is undefined, the selected launch
 * mode is based on the ROMI_DASHBOARD_LAUNCH_MODE environment variable, the values can be one of
 *
 *   * none
 *   * local
 *
 * Defaults to `local`.
 *
 * If the value is `local`, a singleton instance is used and signal handlers are installed.
 * @see LocalLauncher#instance
 * @param launchMode
 */
export function makeLauncher(launchMode?: LaunchMode): Launcher {
  if (!launchMode) {
    if (process.env.ROMI_DASHBOARD_LAUNCH_MODE === 'none') {
      launchMode = LaunchMode.None;
    } else {
      launchMode = LaunchMode.LocalSingleton;
    }
  }

  switch (launchMode) {
    case LaunchMode.None:
      return new StubLauncher();
    case LaunchMode.Local:
      return new LocalLauncher();
    case LaunchMode.LocalSingleton:
      return LocalLauncher.instance;
    default:
      throw new Error('unknown launch mode');
  }
}

/**
 * Help to launch and kill all required RMF processes. Assumes required rmf components are installed
 * and launches them locally.
 */
export class LocalLauncher {
  /**
   * Singleton instance of rmfLauncher, signal handlers are installed to cleanup processses spawned
   * by this instance.
   *
   * Note: This installs `SIGINT` and `SIGTERM` handlers, these handlers may conflict with existing
   * or future handlers. If there are other signal handlers installed, it is recommended to not use
   * this, the instance is lazy created so no handlers will be installed if this is never called.
   */
  static get instance(): LocalLauncher {
    if (!this._instance) {
      this._instance = new LocalLauncher();
      /**
       * Make sure spawned processes are killed when the program exits.
       */
      const cleanUp = async () => {
        await this._instance?.kill();
        process.exit();
      };

      process.once('beforeExit', cleanUp);
      process.once('exit', cleanUp);
      process.once('SIGINT', cleanUp);
      process.once('SIGTERM', cleanUp);
    }
    return this._instance;
  }

  private static _instance?: LocalLauncher;

  async launch(): Promise<void> {
    if (this._launched) {
      return;
    }

    const headless = !process.env.ROMI_DASHBOARD_NO_HEADLESS;
    const officeDemoArgs = ['launch', 'demos', 'office.launch.xml'];
    if (headless) {
      officeDemoArgs.push('headless:=true');
    }
    this._officeDemo = new ManagedProcess('ros2', officeDemoArgs, { stdio: 'inherit' });

    this._soss = ChildProcess.spawn('soss', [`${__dirname}/soss.yaml`], { stdio: 'inherit' });

    this._visualizerServer = new ManagedProcess('ros2', ['launch', 'visualizer', 'server.xml'], {
      stdio: 'inherit',
    });

    const ready = await this._rmfReady();
    if (!ready) {
      throw new Error('unable to detect rmf');
    }

    this._launched = true;
  }

  async kill(): Promise<void> {
    await Promise.all([
      this._officeDemo?.kill('SIGINT'),
      this._soss && this._killProcess(this._soss),
      this._visualizerServer?.kill('SIGINT'),
    ]);
    this._officeDemo = undefined;
    this._soss = undefined;
    this._visualizerServer = undefined;
    this._launched = false;
  }

  private _launched = false;
  private _officeDemo?: ManagedProcess;
  private _soss?: ChildProcess.ChildProcess;
  private _visualizerServer?: ManagedProcess;

  private async _killProcess(
    proc: ChildProcess.ChildProcess,
    signal?: NodeJS.Signals,
  ): Promise<void> {
    if (proc.killed) {
      return Promise.resolve();
    }
    return new Promise(res => {
      proc.once('exit', res);
      proc.kill(signal);
    });
  }

  private async _rmfReady(timeout: number = 30000): Promise<boolean> {
    const ros2Echo = ChildProcess.spawn('ros2', [
      'topic',
      'echo',
      'fleet_states',
      'rmf_fleet_msgs/msg/FleetState',
    ]);
    if (!ros2Echo) {
      return false;
    }

    return new Promise(res => {
      const timer = setTimeout(() => {
        ros2Echo && ros2Echo.kill();
        res(false);
      }, timeout);
      ros2Echo.stdout.once('data', () => {
        ros2Echo.kill();
        clearTimeout(timer);
        res(true);
      });
    });
  }
}

/**
 * A wrapper around soss process that helps manages spawned process and their subprocesses' life.
 * It spawns processes in their own process groups. The `kill` method will kill the process and
 * all its childrens.
 *
 * This is needed especially for `ros2 launch` because newer versions of it no longer propagate
 * signals to its children. So killing `ros2 launch` will leave zombie processes.
 */
class ManagedProcess {
  get proc(): ChildProcess.ChildProcess {
    return this._proc;
  }

  get alive(): boolean {
    return this._procAlive;
  }

  constructor(
    command: string,
    args: string[],
    options?: Omit<ChildProcess.SpawnOptions, 'detached'>,
  ) {
    this._proc = ChildProcess.spawn(command, args, {
      ...options,
      detached: true,
    });
    this._procAlive = !!this._proc;
    if (this._procAlive) {
      this._proc.once('exit', () => (this._procAlive = false));
    }
  }

  /**
   * Kill the child process and all their childrens.
   */
  async kill(signal?: NodeJS.Signals): Promise<void> {
    if (!this._procAlive) {
      return;
    }

    return new Promise(res => {
      this._proc.once('exit', res);
      process.kill(-this._proc.pid, signal);
    });
  }

  private _proc: ChildProcess.ChildProcess;
  private _procAlive = false;
}

/**
 * Stub launcher that does not do anything, useful in dev cycle when you want to manage the rmf
 * processes manually.
 */
export class StubLauncher {
  async launch(): Promise<void> {}
  async kill(): Promise<void> {}
}
