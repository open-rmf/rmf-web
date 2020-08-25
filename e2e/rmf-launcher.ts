import * as ChildProcess from 'child_process';

/**
 * Help to launch and kill all required RMF processes.
 */
export class RmfLauncher {
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
    this._officeDemo = new ManagedProcess('ros2', officeDemoArgs, { stdio: 'inherit' }, 'SIGINT');

    this._soss = ChildProcess.spawn('soss', [`${__dirname}/soss.yaml`], { stdio: 'inherit' });

    this._visualizerServer = new ManagedProcess(
      'ros2',
      ['launch', 'visualizer', 'server.xml'],
      { stdio: 'inherit' },
      'SIGINT',
    );

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
      this._officeDemo?.kill(),
      this._soss && this._killProcess(this._soss),
      this._visualizerServer?.kill(),
    ]);
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
 * This is needed espacially for `ros2 launch` because newer versions of it no longer propagate
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
    killSignal?: NodeJS.Signals,
  ) {
    this._killSignal = killSignal;
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
  async kill(): Promise<void> {
    if (!this._procAlive) {
      return;
    }

    return new Promise(res => {
      this._proc.once('exit', res);
      process.kill(-this._proc.pid, this._killSignal);
    });
  }

  private _proc: ChildProcess.ChildProcess;
  private _procAlive = false;
  private _killSignal?: NodeJS.Signals;
}

async function rmfReady(timeout: number = 30000): Promise<boolean> {
  const ros2Echo = new ManagedProcess('ros2', [
    'topic',
    'echo',
    'fleet_states',
    'rmf_fleet_msgs/msg/FleetState',
  ]);
  if (!ros2Echo.alive) {
    return false;
  }

  return new Promise(res => {
    const timer = setTimeout(() => {
      ros2Echo && ros2Echo.kill();
      res(false);
    }, timeout);
    ros2Echo.proc.stdout!.once('data', () => {
      ros2Echo.kill();
      clearTimeout(timer);
      res(true);
    });
  });
}
