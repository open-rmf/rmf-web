import * as ChildProcess from 'child_process';

export class RmfLauncher {
  constructor(launchMode?: LaunchMode) {
    if (!launchMode) {
      if (process.env.ROMI_DASHBOARD_DOCKER_LAUNCH) {
        this._launchMode = LaunchMode.Docker;
      } else if (process.env.ROMI_DASHBOARD_NO_LAUNCH) {
        this._launchMode = LaunchMode.None;
      } else {
        this._launchMode = LaunchMode.Local;
      }
    } else {
      this._launchMode = launchMode;
    }

    switch (this._launchMode) {
      case LaunchMode.None:
        this._launcher = new StubLauncher();
        break;
      case LaunchMode.Docker:
        this._launcher = new DockerLauncher();
        break;
      case LaunchMode.Local:
        this._launcher = new LocalLauncher();
        break;
    }
  }

  async launch(): Promise<void> {
    return this._launcher.launch();
  }

  async kill(): Promise<void> {
    return this._launcher.kill();
  }

  private _launchMode: LaunchMode;
  private _launcher: Launcher;
}

enum LaunchMode {
  None,
  Local,
  Docker,
}

interface Launcher {
  launch(): Promise<void>;
  kill(): Promise<void>;
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
  static get instance(): RmfLauncher {
    if (!this._instance) {
      this._instance = new RmfLauncher();
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

  private static _instance?: RmfLauncher;

  async launch(): Promise<void> {
    if (process.env.ROMI_DASHBOARD_NO_LAUNCH) {
      return;
    }

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

    const ready = await rmfReady();
    if (!ready) {
      throw new Error('unable to detect rmf');
    }

    this._launched = true;
  }

  async kill(): Promise<void> {
    if (process.env.ROMI_DASHBOARD_NO_LAUNCH) {
      return;
    }

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

async function rmfReady(timeout: number = 30000): Promise<boolean> {
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

/**
 * Launches rmf components in docker containers.
 */
class DockerLauncher {
  async launch(): Promise<void> {
    if (this._proc) {
      return;
    }
    this._proc = ChildProcess.spawn(
      `${__dirname}/../scripts/dockert`,
      [
        'docker-compose',
        '-f',
        `${__dirname}/../docker/docker-compose.yml`,
        'up',
        'office-demo',
        'trajectory-server',
        'soss',
      ],
      { stdio: 'inherit' },
    );
    process.on('beforeExit', () => {
      this.kill();
    });
  }

  async kill(): Promise<void> {
    if (!this._proc) {
      return;
    }
    this._proc.kill();
    return new Promise(res => {
      if (!this._proc) {
        return res();
      }
      this._proc.once('exit', res);
    });
  }

  private _proc?: ChildProcess.ChildProcess;
}

/**
 * Stub launcher that does not do anything, useful in dev cycle when you want to manage the rmf
 * processes manually.
 */
class StubLauncher {
  async launch(): Promise<void> {}
  async kill(): Promise<void> {}
}
