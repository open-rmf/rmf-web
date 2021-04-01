/**
 * Simple script that runs npm install or ci depending on the CI flag for each package if the node_modules dir does not exist.
 * Usage: bootstrap.js [package]...
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

// hardcoded for now
const deps = {
  'packages/dashboard': [
    'packages/ros2-bridge',
    'packages/react-components',
    'packages/api-client',
    'packages/rmf-auth',
  ],
  'packages/reporting': ['packages/react-components', 'packages/rmf-auth'],
  'packages/api-client': ['packages/api-server'],
};

const allPackages = [
  '.',
  'packages/ros2-bridge',
  'packages/react-components',
  'packages/rmf-auth',
  'packages/reporting',
  'packages/dashboard',
  'packages/api-server',
  'packages/api-client',
];
const scope = process.argv.length > 2 ? process.argv.slice(2) : allPackages;
const verb = process.env['CI'] ? 'ci' : 'install';
const targets = new Set(['.']);
scope.forEach((pkg) => {
  if (deps[pkg]) {
    deps[pkg].forEach((p) => targets.add(p));
  }
  targets.add(pkg);
});
targets.forEach((pkg) => {
  const cwd = `${__dirname}/../${pkg}`;
  const result = child_process.spawnSync('npm', [verb], { stdio: 'inherit', cwd });
  if (result.status !== 0) {
    process.exit(result.status);
  }
});
