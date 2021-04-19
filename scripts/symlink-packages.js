const fs = require('fs');
const { allPackages } = require('./packages');

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
