const fs = require('fs');

if (!process.env.WORLD_NAME) {
  console.error('You must assign a world name');
  return;
}

const worldName = process.env.WORLD_NAME;

const getPackageLocation = (packageName) => {
  if (!packageName) {
    throw new Error('You must provide a package name');
  }
  return `${process.env.RMF_PACKAGES}${packageName}/`;
};

const rmfDashboardResources = `${getPackageLocation('rmf_demos_dashboard_resources')}${worldName}/`;

const resources = `{"path": "${rmfDashboardResources}"}`;

fs.writeFile('.resources.json', resources, function (err) {
  if (err) {
    throw err;
  }
  console.log(`File .resources.json was created successfully.`);
});
