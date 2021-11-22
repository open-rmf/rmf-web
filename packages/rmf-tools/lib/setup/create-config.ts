const chalk = require('chalk');
const inquirer = require('inquirer');
const { exec } = require('child_process');
const fs = require('fs');

export const createConfigFile = () => {
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
        validate: (input: string) => /^[1-2-3-]$/.test(input) || 'Invalid option',
      },
      {
        name: 'REPO',
        message: `Set REPO. Example: https://github.com/open-rmf/rmf_demos.git`,
        when: (keys: { [key: string]: string }) => keys['GET_OR_COPY'] === '1',
        validate: (input: string) =>
          /[(http(s)?):\/\/(www\.)?a-zA-Z0-9@:%._\+~#=]{2,256}\.[a-z]{2,6}\b([-a-zA-Z0-9@:%_\+.~#?&//=]*)/.test(
            input,
          ) || 'Not a valid URL',
      },
      {
        name: 'BRANCH',
        message: `Set BRANCH. Example: main`,
        when: (keys: { [key: string]: string }) => !!keys['REPO'],
        default: 'main',
      },
      {
        name: 'FOLDER',
        message: `Set FOLDER. Leave it empty to clone the whole project. Example: rmf_demos_dashboard_resources/office/`,
        when: (keys: { [key: string]: string }) => !!keys['BRANCH'],
        default: '',
      },
      {
        name: 'COPY',
        message: 'Set absolute PATH',
        when: (keys: { [key: string]: string }) => keys['GET_OR_COPY'] === '2',
        validate: (input: string) => /^(\/[^\/]+){0,20}\/?$/gm.test(input) || 'Not a valid PATH',
      },
      {
        name: 'ROS_PACKAGE',
        message: 'ROS package',
        when: (keys: { [key: string]: string }) => keys['GET_OR_COPY'] === '3',
        default: 'rmf_demos_dashboard_resources',
      },
      {
        name: 'ROS_PACKAGE_PATH',
        message: 'path',
        when: (keys: { [key: string]: string }) => !!keys['ROS_PACKAGE'],
        default: 'office',
      },
    ])
    .then((keys: { [key: string]: string }) => {
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
        function (err: Error) {
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
    .catch((error: Error) => console.error(error));
};
