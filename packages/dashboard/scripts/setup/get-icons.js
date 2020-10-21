'use strict';
const { exec, execSync } = require('child_process');
const fs = require('fs');
const chalk = require('chalk');
const path = require('path');

let file = './.resources.json';
if (process.env.RESOURCE_FILE) {
  file = process.env.RESOURCE_FILE;
}

const fileExists = fs.existsSync(file);
if (!fileExists) {
  console.log(`Configuration file ${file} not exists`);
  return;
}

const resourcesData = JSON.parse(fs.readFileSync(file));

const iconFolder = 'public/assets/icons/';

if (!resourcesData.hasOwnProperty('repoUrl') && !resourcesData.hasOwnProperty('path')) {
  return;
}

class IconManagerBase {
  constructor(resourcesData, iconFolder, tempFolder) {
    this.resourcesData = resourcesData;
    this.iconFolder = iconFolder;
    this.tempFolder = tempFolder;
    this.rootPath = '../../';
  }
}

/**
 * Implementation for git sparse-checkout, on versions of git greater or equal to 2.25.
 */
class SparseCheckoutGitV225 extends IconManagerBase {
  constructor(resourcesData, iconFolder, tempFolder) {
    super(resourcesData, iconFolder, tempFolder);
  }

  execute = () => {
    execSync(
      `git clone "${this.resourcesData.repoUrl}" --no-checkout  --depth=1 --single-branch --branch ${this.resourcesData.branch} ${this.tempFolder} -o repo`,
      {
        stdio: [0, 1, 2],
        cwd: path.resolve(__dirname, this.rootPath),
      },
    );

    execSync(
      `git sparse-checkout init --cone &&
            git config core.sparseCheckout 1 &&
            git sparse-checkout set ${this.resourcesData.folder} &&
            git checkout`,
      {
        stdio: [0, 1, 2],
        cwd: path.resolve(__dirname, `../../${this.tempFolder}`),
      },
    );
  };
}

/**
 * Implementation for git sparse-checkout, on versions of git lesser than 2.25.
 */
class SparseCheckoutGitV217 extends IconManagerBase {
  constructor(resourcesData, iconFolder, tempFolder) {
    super(resourcesData, iconFolder, tempFolder);
    this.tempFolderLocation = `../../${tempFolder}`;
  }

  execute = () => {
    execSync(`git init && git remote add -f resources "${this.resourcesData.repoUrl}"`, {
      stdio: [0, 1, 2],
      cwd: path.resolve(__dirname, this.tempFolderLocation),
    });
    execSync(
      `git config core.sparseCheckout true && echo ${this.resourcesData.folder} >> .git/info/sparse-checkout`,
      {
        stdio: [0, 1, 2],
        cwd: path.resolve(__dirname, this.tempFolderLocation),
      },
    );
    execSync(`git pull --depth=1 resources ${this.resourcesData.branch}`, {
      stdio: [0, 1, 2],
      cwd: path.resolve(__dirname, this.tempFolderLocation),
    });
  };
}

class IconManager extends IconManagerBase {
  constructor(resourcesData, iconFolder, tempFolder) {
    super(resourcesData, iconFolder, tempFolder);
  }

  getGitMinorVersion = (rawGitVersion) => {
    const gitVersion = rawGitVersion.split(' ')[2];
    const gitMinorVersion = gitVersion.split('.')[1];
    return parseInt(gitMinorVersion);
  };

  cloneSpecificFolder = () => {
    this.createTmpFolder();
    exec(`git --version`, (error, stdout, stderr) => {
      if (error) {
        console.error(chalk`{red exec error: ${error}}`);
        return;
      }
      const cloneImplementation =
        this.getGitMinorVersion(stdout) < 25 ? SparseCheckoutGitV217 : SparseCheckoutGitV225;

      new cloneImplementation(this.resourcesData, this.iconFolder, this.tempFolder).execute();
      this.moveFromTmpFolderToIconFolder();
      this.removeTmpFolder();
    });
  };

  cloneRepo = () => {
    execSync(
      `git clone "${this.resourcesData.repoUrl}" --depth=1 --single-branch --branch ${this.resourcesData.branch} ${this.iconFolder} -o repo`,
      {
        stdio: [0, 1, 2], // we need this so node will print the command output
        cwd: path.resolve(__dirname, this.rootPath), // path to where you want to save the file
      },
    );
  };

  removeTmpFolder = () => {
    exec(`[ -d "${this.tempFolder}" ] && rm -rf ${this.tempFolder}`);
  };

  createTmpFolder = () => {
    exec(`[ -d "${this.tempFolder}" ] && rm -rf ${this.tempFolder}`);
    exec(`mkdir -p ${this.tempFolder}`);
  };

  moveFromTmpFolderToIconFolder = () => {
    exec(`cp -r ${this.tempFolder}/${this.resourcesData.folder}/* ${this.iconFolder}`, {
      stdio: [0, 1, 2],
      cwd: path.resolve(__dirname, this.rootPath),
    });
  };

  copyFromLocalDirectory = () => {
    exec(`cp -r ${this.resourcesData.path}* ${this.iconFolder}`, (error, stdout, stderr) => {
      if (error) {
        console.error(chalk`{red exec error: ${error}}`);
        return;
      }

      if (stderr) {
        console.error(`stderr: ${stderr}`);
        return;
      }

      console.log(`stdout: ${stdout}`);
      console.log(
        chalk`{green The icons have been successfully obtained. Check public/assets/icons/}`,
      );
    });
  };
}

const getIcons = () => {
  const tempFolder = 'tmp';
  const iconManager = new IconManager(resourcesData, iconFolder, tempFolder);

  if (resourcesData.hasOwnProperty('repoUrl') || resourcesData.hasOwnProperty('path')) {
    exec(`[ -d "${iconFolder}" ] && rm -rf ${iconFolder}`);
    exec(`mkdir -p ${iconFolder}`);
  }

  if (resourcesData.hasOwnProperty('repoUrl')) {
    // If we don't want to clone a specific folder of the repo, it'll clone the whole repo
    if (!resourcesData.folder) {
      iconManager.cloneRepo();
      return;
    }
    iconManager.cloneSpecificFolder();
  } else {
    iconManager.copyFromLocalDirectory();
  }
};

getIcons();
