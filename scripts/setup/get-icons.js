'use strict';
const { exec, execSync } = require('child_process');
const fs = require('fs');
const chalk = require('chalk');
const path = require('path');

const rootPath = path.resolve(__dirname);
const fileExists = fs.existsSync('./.resources.json');
if (!fileExists) {
  console.log('File ./.resources.json not exists');
  return;
}

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

let rawdata = fs.readFileSync('./.resources.json');
let data = JSON.parse(rawdata);

if (data.hasOwnProperty('repoUrl')) {
  exec('[ -d "public/assets/icons/" ] && rm -rf public/assets/icons');
  exec('mkdir -p public/assets/icons');
  // TODO remove --branch when osrf/rmf_demos#109 is merged or add support for branches on prompt
  execSync(
    `git clone "${data.repoUrl}" --no-checkout  --depth=1 --single-branch --branch feat/createResourceFolder public/assets/icons/ -o repo`,
    {
      stdio: [0, 1, 2], // we need this so node will print the command output
      cwd: path.resolve(__dirname, '../../'), // path to where you want to save the file
    },
  );

  execSync(
    `git sparse-checkout init --cone &&
      git config core.sparseCheckout 1 &&
      git sparse-checkout set ${data.folder} &&
      git checkout`,
    {
      stdio: [0, 1, 2],
      cwd: path.resolve(__dirname, '../../public/assets/icons/'),
    },
  );

  exec(`cp -r rmf_dashboard_resources/office/* . && rm -rf rmf_dashboard_resources/`, {
    stdio: [0, 1, 2],
    cwd: path.resolve(__dirname, '../../public/assets/icons/'),
  });
} else if (data.hasOwnProperty('path')) {
  exec('[ -d "public/assets/icons/" ] && rm -rf public/assets/icons');
  exec('mkdir -p public/assets/icons');
  exec(`cp -r ${data.path}* public/assets/icons/`, (error, stdout, stderr) =>
    callback(error, stdout, stderr),
  );
}
