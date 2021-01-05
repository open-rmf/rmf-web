/**
 * Simple script that runs npm install or ci depending on the CI flag for each package if the node_modules dir does not exist.
 *
 * The previous implementation chains npm install/ci with the "preinstall" script, this is bad for
 * several reasons.
 *   1. npm exports some env variables, chaining npm in scripts causes the commands to inherit
 *     the context of the parent npm, this may cause corrupted installs or undefined behaviors.
 *   2. The "preinstall" script is always ran in a npm install/ci, even for transitive dependencies,
 *     this causes all packages to be not "publishable" as those "preinstall" scripts shouldn't be
 *     there on an "external" install.
 */
const child_process = require('child_process');
const fs = require('fs');

// Install order matters, a package's dependencies needs to be installed before itself.
const packages = ['.', 'packages/ros2-bridge', 'packages/react-components', 'packages/dashboard'];

const verb = process.env['CI'] ? 'ci' : 'install';
packages.forEach((pkg) => {
  const cwd = `${__dirname}/../${pkg}`;
  try {
    fs.accessSync(`${cwd}/node_modules`);
  } catch (e) {
    if (e.code === 'ENOENT') {
      child_process.spawnSync('npm', [verb], { stdio: 'inherit', cwd });
    } else {
      throw e;
    }
  }
});
