const { spawn } = require('child_process');
const { mkdirSync } = require('fs');

const LaunchMode = {
  None: 0,
  LocalSingleton: 1,
};

/**
 * Help to launch and kill all required RMF processes. Assumes required rmf components are installed
 * and launches them locally.
 */
exports.LocalLauncher = class {
  /**
   * Singleton instance of rmfLauncher, signal handlers are installed to cleanup processses spawned
   * by this instance.
   *
   * Note: This installs `SIGINT` and `SIGTERM` handlers, these handlers may conflict with existing
   * or future handlers. If there are other signal handlers installed, it is recommended to not use
   * this, the instance is lazy created so no handlers will be installed if this is never called.
   */
  static get instance() {
    if (!this._instance) {
      this._instance = new exports.LocalLauncher();
      /**
       * Make sure spawned processes are killed when the program exits.
       */
      const cleanUp = async () => {
        await this._instance?.kill();
      };

      process.once('beforeExit', cleanUp);
      process.once('exit', cleanUp);
      process.once('SIGINT', cleanUp);
      process.once('SIGTERM', cleanUp);
    }
    return this._instance;
  }

  static _instance;

  async launch(timeout = 30000) {
    if (this._launched) {
      return;
    }

    const headless = !process.env.RMF_DASHBOARD_NO_HEADLESS;
    const demoPkg = process.env.RMF_DASHBOARD_DEMO_PACKAGE || 'rmf_demos_gz';
    const demoMap = process.env.RMF_DASHBOARD_DEMO_MAP || 'office.launch.xml';
    const demoArgs = ['launch', demoPkg, demoMap, 'server_uri:=ws://localhost:8000/_internal'];
    if (headless) {
      demoArgs.push('headless:=true');
    }
    mkdirSync(`${__dirname}/.rmf`, { recursive: true });
    this._rmfDemo = new ManagedProcess('ros2', demoArgs, {
      stdio: 'inherit',
      cwd: `${__dirname}/.rmf`,
    });

    const ready = await this._rmfReady(timeout);
    if (!ready) {
      throw new Error('unable to detect rmf');
    }

    this._launched = true;
  }

  async kill() {
    await Promise.all([this._rmfDemo?.kill('SIGINT')]);
    this._rmfDemo = undefined;
    this._launched = false;
  }

  _launched = false;
  _rmfDemo;

  async _rmfReady(timeout) {
    const ros2Echo = spawn('ros2', [
      'topic',
      'echo',
      'fleet_states',
      'rmf_fleet_msgs/msg/FleetState',
    ]);
    if (!ros2Echo) {
      return false;
    }

    return new Promise((res) => {
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
};

/**
 * A wrapper around child process that helps manages spawned process and their subprocesses' life.
 * It spawns processes in their own process groups. The `kill` method will kill the process and
 * all its childrens.
 *
 * This is needed especially for `ros2 launch` because newer versions of it no longer propagate
 * signals to its children. So killing `ros2 launch` will leave zombie processes.
 */
class ManagedProcess {
  get proc() {
    return this._proc;
  }

  get alive() {
    return this._procAlive;
  }

  constructor(command, args, options) {
    this._proc = spawn(command, args, {
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
  async kill(signal) {
    if (!this._procAlive) {
      return;
    }

    return new Promise((res) => {
      this._proc.once('exit', res);
      process.kill(-this._proc.pid, signal);
    });
  }

  _proc;
  _procAlive = false;
}

/**
 * Stub launcher that does not do anything, useful in dev cycle when you want to manage the rmf
 * processes manually.
 */
exports.StubLauncher = class {
  async launch() {}
  async kill() {}
};

/**
 * Creates a launcher based on the RMF_LAUNCH_MODE environment variable. Value can be one of:
 *
 *   * none
 *   * local
 *
 * Defaults to `local`.
 *
 * If the value is `local`, a singleton instance is used and signal handlers are installed.
 */
exports.makeLauncher = () => {
  let launchMode = LaunchMode.LocalSingleton;

  if (process.env.RMF_LAUNCH_MODE === 'none') {
    launchMode = LaunchMode.None;
  }

  switch (launchMode) {
    case LaunchMode.None:
      return new exports.StubLauncher();
    case LaunchMode.LocalSingleton:
      return exports.LocalLauncher.instance;
    default:
      throw new Error('unknown launch mode');
  }
};
