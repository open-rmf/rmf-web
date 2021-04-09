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
const fs = require('fs');

// hardcoded for now
const deps = {
  'packages/dashboard': [
    'packages/react-components',
    'packages/api-client',
    'packages/rmf-auth',
    'packages/rmf-models',
  ],
  'packages/react-components': ['packages/api-client', 'packages/rmf-models'],
  'packages/reporting': ['packages/react-components', 'packages/rmf-auth'],
  'packages/api-client': ['packages/rmf-models'],
};

function getDeps(pkg) {
  const recur = (pkg, cur) => {
    if (deps[pkg]) {
      deps[pkg].forEach((p) => {
        recur(p, cur);
      });
    }
    cur.add(pkg);
    return cur;
  };
  return recur(pkg, new Set());
}

const allPackages = [
  'packages/ros2-bridge',
  'packages/react-components',
  'packages/rmf-auth',
  'packages/reporting',
  'packages/dashboard',
  'packages/api-server',
  'packages/api-client',
  'packages/rmf-models',
];
const scope = process.argv.length > 2 ? process.argv.slice(2) : allPackages;
const verb = process.env['CI'] ? 'ci' : 'install';

const result = child_process.spawnSync('npm', [verb], { stdio: 'inherit', cwd: `${__dirname}/..` });
if (result.status !== 0) {
  process.exit(result.status);
}
allPackages.forEach((pkg) => {
  const packageJson = JSON.parse(fs.readFileSync(`${pkg}/package.json`));
  const pkgName = packageJson.name;
  try {
    fs.unlinkSync(`node_modules/${pkgName}`);
  } catch (e) {
    if (e.code !== 'ENOENT') {
      throw e;
    }
  }
  fs.symlinkSync(`../${pkg}`, `node_modules/${pkgName}`);
  console.log(`symlinked ${pkgName}`);
});

const targets = new Set();
scope.forEach((pkg) => {
  getDeps(pkg).forEach((p) => targets.add(p));
});
targets.forEach((pkg) => {
  const cwd = `${__dirname}/../${pkg}`;
  const result = child_process.spawnSync('npm', [verb], { stdio: 'inherit', cwd });
  if (result.status !== 0) {
    process.exit(result.status);
  }
});
