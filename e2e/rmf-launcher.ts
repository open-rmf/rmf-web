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
    this._officeDemo = ChildProcess.spawn('ros2', officeDemoArgs);
    this._soss = ChildProcess.spawn('soss', [`${__dirname}/soss.yaml`]);
    this._visualizerServer = ChildProcess.spawn('ros2', ['launch', 'visualizer', 'server.xml']);
    this._ros2Echo = ChildProcess.spawn('ros2', [
      'topic',
      'echo',
      'fleet_states',
      'rmf_fleet_msgs/msg/FleetState',
    ]);

    process.once('beforeExit', async () => {
      await this.kill();
    });

    // wait for the rmf to be ready
    await new Promise(res => {
      if (!this._ros2Echo) {
        res();
        return;
      }
      this._ros2Echo.stdout.once('data', () => {
        res();
      });
    });
    await this._killProcess(this._ros2Echo);
  }

  async kill(): Promise<void> {
    if (process.env.ROMI_DASHBOARD_NO_LAUNCH) {
      return;
    }

    await Promise.all([
      this._officeDemo && this._killProcess(this._officeDemo, 'SIGINT'), // doesn't clean up properly with SIGTERM
      this._soss && this._killProcess(this._soss),
      this._visualizerServer && this._killProcess(this._visualizerServer, 'SIGINT'), // doesn't clean up properly with SIGTERM
      this._ros2Echo && this._killProcess(this._ros2Echo),
    ]);
  }

  private _launched = false;
  private _officeDemo?: ChildProcess.ChildProcessWithoutNullStreams;
  private _soss?: ChildProcess.ChildProcessWithoutNullStreams;
  private _visualizerServer?: ChildProcess.ChildProcessWithoutNullStreams;
  private _ros2Echo?: ChildProcess.ChildProcessWithoutNullStreams;

  private async _killProcess(
    proc: ChildProcess.ChildProcess,
    signal?: NodeJS.Signals,
  ): Promise<void> {
    if (proc.killed) {
      return Promise.resolve();
    }
    proc.kill(signal);
    return new Promise(res => proc.once('exit', res));
  }
}
