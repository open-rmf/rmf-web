const chalk = require('chalk');
const fs = require('fs');
const { getIcons, createConfigFile } = require('rmf-tools');

const ProjectDir = __dirname.slice(0, __dirname.length - '/scripts/setup'.length);
const resourcesPath = process.env.RMF_DASHBOARD_RESOURCES_FILE || `${ProjectDir}/.resources.json`;
const configExists = fs.existsSync(resourcesPath);

(async () => {
  if (configExists) {
    console.log(chalk`{blue Using existing resources config}`);
  } else {
    await createConfigFile();
  }

  const resourcesData = JSON.parse(fs.readFileSync(resourcesPath));
  try {
    getIcons(resourcesData, 'dashboard');
  } catch (e) {
    console.error(e);
    process.exit(e.status);
  }
})();
