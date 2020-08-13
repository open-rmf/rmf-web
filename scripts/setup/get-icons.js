'use strict';
const { exec, execSync } = require('child_process');
const fs = require('fs');
const chalk = require('chalk');
const path = require('path');

const fileExists = fs.existsSync('./.resources.json');
if (!fileExists) {
  console.log('File ./.resources.json not exists');
  return;
}

const resourcesData = JSON.parse(fs.readFileSync('./.resources.json'));
const iconFolder = 'public/assets/icons/';

const callback = (error, stdout, stderr) => {
  if (error) {
    console.error(chalk`{red exec error: ${error}}`);
    return;
  }

  if (stderr) {
    console.error(`stderr: ${stderr}`);
    return;
  }

  console.log(`stdout: ${stdout}`);
  console.log(chalk`{green The icons have been successfully obtained. Check public/assets/icons/}`);
};

if (resourcesData.hasOwnProperty('repoUrl') || resourcesData.hasOwnProperty('path')) {
  exec(`[ -d "${iconFolder}" ] && rm -rf ${iconFolder}`);
  exec(`mkdir -p ${iconFolder}`);
}

if (resourcesData.hasOwnProperty('repoUrl')) {
  if (!resourcesData.folder) {
    execSync(
      `git clone "${resourcesData.repoUrl}" --depth=1 --single-branch --branch ${resourcesData.branch} ${iconFolder} -o repo`,
      {
        stdio: [0, 1, 2], // we need this so node will print the command output
        cwd: path.resolve(__dirname, '../../'), // path to where you want to save the file
      },
    );
    return;
  }
  const tempFolder = 'tmp';

  exec(`[ -d "${tempFolder}" ] && rm -rf ${tempFolder}`);
  exec(`mkdir -p ${tempFolder}`);

  execSync(
    `git clone "${resourcesData.repoUrl}" --no-checkout  --depth=1 --single-branch --branch ${resourcesData.branch} ${tempFolder} -o repo`,
    {
      stdio: [0, 1, 2],
      cwd: path.resolve(__dirname, '../../'),
    },
  );

  execSync(
    `git sparse-checkout init --cone &&
      git config core.sparseCheckout 1 &&
      git sparse-checkout set ${resourcesData.folder} &&
      git checkout`,
    {
      stdio: [0, 1, 2],
      cwd: path.resolve(__dirname, `../../${tempFolder}`),
    },
  );
  exec(`cp -r ${tempFolder}/${resourcesData.folder}/* ${iconFolder}`, {
    stdio: [0, 1, 2],
    cwd: path.resolve(__dirname, `../../`),
  });

  exec(`[ -d "${tempFolder}" ] && rm -rf ${tempFolder}`);
} else if (resourcesData.hasOwnProperty('path')) {
  exec(`cp -r ${resourcesData.path}* ${iconFolder}`, (error, stdout, stderr) =>
    callback(error, stdout, stderr),
  );
}
