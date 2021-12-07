const fs = require('fs');

const root = `${__dirname}/..`;
const rootPackageJson = JSON.parse(fs.readFileSync(`${root}/package.json`));
const packages = rootPackageJson.workspaces;
const packageJson = packages.reduce((acc, pkg) => {
  const p = JSON.parse(fs.readFileSync(`${root}/${pkg}/package.json`));
  acc[p.name] = p;
  return acc;
}, {});

function getDirectDeps(pkg) {
  const deps = { ...packageJson[pkg].dependencies, ...packageJson[pkg].devDependencies };
  return Object.keys(deps).filter((dep) => {
    return dep in packageJson;
  });
}

function getAllDeps(pkgs, acc = new Set()) {
  pkgs.forEach((d) => {
    // move this to the end of the set
    acc.delete(d);
    acc.add(d);
  });
  const directDeps = pkgs.flatMap((pkg) => getDirectDeps(pkg));
  if (directDeps.length === 0) {
    return acc;
  }
  directDeps.forEach((d) => {
    // move this to the end of the set
    acc.delete(d);
    acc.add(d);
  });
  return getAllDeps(directDeps, acc);
}

const args = process.argv.slice(2);
const flags = args.filter((arg) => arg.startsWith('-'));
const targets = args.filter((arg) => !arg.startsWith('-'));
const allDeps = (() => {
  if (flags.indexOf('--only-direct') !== -1) {
    return [...targets, ...targets.flatMap((t) => getDirectDeps(t))];
  } else {
    return Array.from(getAllDeps(targets));
  }
})();

const workspaceArgs = [];
while (allDeps.length > 0) {
  workspaceArgs.push(`-w='${allDeps.pop()}'`);
}
console.log(workspaceArgs.join(' '));
