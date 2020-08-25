import { RmfLauncher } from './rmf-launcher';

const launcher = new RmfLauncher();
launcher.launch();

/**
 * Make sure spawned processes are killed when the program exits.
 */
async function cleanUp(): Promise<never> {
  await launcher.kill();
  process.exit();
}

process.once('beforeExit', cleanUp);
process.once('exit', cleanUp);
process.once('SIGINT', cleanUp);
process.once('SIGTERM', cleanUp);
