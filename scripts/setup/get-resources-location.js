const fs = require('fs');

const getDashboardResourcesLocation = () => {
  return '/opt/rmf/share/rmf_demos_dashboard_resources/';
};

const rmfDashboardResources = getDashboardResourcesLocation() + 'office/';

const resources = `{"path": "${rmfDashboardResources}"}`;

fs.writeFile('.resources.json', resources, function (err) {
  if (err) {
    throw err;
  }
  console.log(`File .resources.json was created successfully.`);
});
