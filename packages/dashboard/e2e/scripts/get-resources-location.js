const fs = require('fs');

if (!process.env.WORLD_NAME) {
  console.error('You must assign a world name');
  return;
}
const worldName = process.env.WORLD_NAME;

class Ament {
  findPackage(packageName) {
    this._isEmpty(packageName);
    return this._getPackageLocation(packageName);
  }

  _isEmpty(packageName) {
    if (!packageName) {
      throw new Error('You must provide a package name');
    }
  }

  _getPackageLocation(packageName) {
    const paths = this._getAmentPrefixPath().split(':');
    // Is package inside?
    for (let p in paths) {
      const path = paths[p];
      const location = `${path}/share/${packageName}/`;
      const directoryExists = fs.existsSync(location);
      if (directoryExists) {
        return location;
      }
    }
  }

  _getAmentPrefixPath() {
    const amentPath = process.env.AMENT_PREFIX_PATH;
    if (!amentPath) {
      throw new Error('Cannot found AMENT_PREFIX_PATH');
    }
    return process.env.AMENT_PREFIX_PATH;
  }
}

const ament = new Ament();
const rmfDashboardResources = `${ament.findPackage('rmf_demos_dashboard_resources')}${worldName}/`;

fs.writeFile('../.resources.json', `{"path": "${rmfDashboardResources}"}`, function (err) {
  if (err) {
    throw err;
  }
  console.log(`File .resources.json was created successfully.`);
});
