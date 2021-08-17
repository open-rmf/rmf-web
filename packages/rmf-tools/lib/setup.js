const chalk = require('chalk');
const inquirer = require('inquirer');
const { exec } = require('child_process');
const fs = require('fs');
const { getIcons } = require('./get-icons');

const ProjectDir = __dirname.slice(0, __dirname.length - '/scripts/setup'.length);
const resourcesPath = process.env.RMF_DASHBOARD_RESOURCES_FILE || `${ProjectDir}/.resources.json`;
const configExists = fs.existsSync(resourcesPath);

const createConfigFile = () => {
  console.log(chalk`
Hello! 👋

{bold.red IMPORTANT} 🚨 You are about to configure the source of the resources for the RMF Dashboard project `);
  console.log(chalk`
You have two options:
{cyan →} 1. {blue get} the assets from a remote GH source.
{cyan →} 2. {blue copy} assets from a specific folder.
{cyan →} 3. {blue get from ws} assets from workspace folder (Do not forget to source rmf_demos project before using this option).
`);

  return inquirer
    .prompt([
      {
        name: 'GET_OR_COPY',
        message: chalk`{blue What option are you going to choose?}`,
        validate: (input) => /^[1-2-3-]$/.test(input) || 'Invalid option',
      },
      {
        name: 'REPO',
        message: `Set REPO. Example: https://github.com/open-rmf/rmf_demos.git`,
        when: (keys) => keys['GET_OR_COPY'] === '1',
        validate: (input) =>
          /[(http(s)?):\/\/(www\.)?a-zA-Z0-9@:%._\+~#=]{2,256}\.[a-z]{2,6}\b([-a-zA-Z0-9@:%_\+.~#?&//=]*)/.test(
            input,
          ) || 'Not a valid URL',
      },
      {
        name: 'BRANCH',
        message: `Set BRANCH. Example: main`,
        when: (keys) => !!keys['REPO'],
        default: 'main',
      },
      {
        name: 'FOLDER',
        message: `Set FOLDER. Leave it empty to clone the whole project. Example: rmf_demos_dashboard_resources/office/`,
        when: (keys) => !!keys['BRANCH'],
        default: '',
      },
      {
        name: 'COPY',
        message: 'Set absolute PATH',
        when: (keys) => keys['GET_OR_COPY'] === '2',
        validate: (input) => /^(\/[^\/]+){0,20}\/?$/gm.test(input) || 'Not a valid PATH',
      },
      {
        name: 'ROS_PACKAGE',
        message: 'ROS package',
        when: (keys) => keys['GET_OR_COPY'] === '3',
        default: 'rmf_demos_dashboard_resources',
      },
      {
        name: 'ROS_PACKAGE_PATH',
        message: 'path',
        when: (keys) => !!keys['ROS_PACKAGE'],
        default: 'office',
      },
    ])
    .then((keys) => {
      let information;
      if (keys.GET_OR_COPY === '1') {
        information = { repoUrl: keys.REPO, folder: keys.FOLDER, branch: keys.BRANCH };
      }
      if (keys.GET_OR_COPY === '2') {
        information = { path: keys.COPY };
      }
      if (keys.GET_OR_COPY === '3') {
        information = { rosPackage: keys.ROS_PACKAGE, path: keys.ROS_PACKAGE_PATH };
      }
      fs.writeFileSync(
        '.resources.json',
        JSON.stringify(information, undefined, 2),
        function (err) {
          if (err) {
            exec('mv .bak-resources.json .resources.json');
            throw err;
          }
          console.log(chalk`{green File .resources.json was created successfully.}`);
          try {
            exec('rm .bak-resources.json');
          } catch (error) {}
        },
      );
    })
    .catch((error) => console.error(error));
};

(async () => {
  if (configExists) {
    console.log(chalk`{blue Using existing resources config}`);
  } else {
    await createConfigFile();
  }

  const resourcesData = JSON.parse(fs.readFileSync(resourcesPath));
  try {
    getIcons(resourcesData);
  } catch (e) {
    console.error(e);
    process.exit(e.status);
  }
})();
