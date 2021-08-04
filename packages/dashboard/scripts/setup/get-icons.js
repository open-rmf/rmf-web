'use strict';

const { exec, execSync } = require('child_process');
const chalk = require('chalk');
const path = require('path');

const ProjectDir = __dirname.slice(0, __dirname.length - '/scripts/setup'.length);
const AssetsPath = `${ProjectDir}/src/assets`;
const TempDir = `${ProjectDir}/tmp`;

class IconManagerBase {
  constructor(resourcesData) {
    this.resourcesData = resourcesData;
  }
}

/**
 * Implementation for git sparse-checkout, on versions of git greater or equal to 2.25.
 */
class SparseCheckoutGitV225 extends IconManagerBase {
  execute = () => {
    execSync(
      `git clone "${this.resourcesData.repoUrl}" --no-checkout  --depth=1 --single-branch --branch ${this.resourcesData.branch} ${TempDir} -o repo`,
      {
        stdio: 'inherit',
      },
    );

    execSync(
      `git sparse-checkout init --cone &&
            git config core.sparseCheckout 1 &&
            git sparse-checkout set ${this.resourcesData.folder} &&
            git checkout`,
      {
        stdio: 'inherit',
        cwd: TempDir,
      },
    );
  };
}

/**
 * Implementation for git sparse-checkout, on versions of git lesser than 2.25.
 */
class SparseCheckoutGitV217 extends IconManagerBase {
  execute = () => {
    execSync(`git init && git remote add -f resources "${this.resourcesData.repoUrl}"`, {
      stdio: 'inherit',
      cwd: path.resolve(__dirname, TempDir),
    });
    execSync(
      `git config core.sparseCheckout true && echo ${this.resourcesData.folder} >> .git/info/sparse-checkout`,
      {
        stdio: 'inherit',
        cwd: path.resolve(__dirname, TempDir),
      },
    );
    execSync(`git pull --depth=1 resources ${this.resourcesData.branch}`, {
      stdio: 'inherit',
      cwd: path.resolve(__dirname, TempDir),
    });
  };
}

class IconManager extends IconManagerBase {
  getGitMinorVersion = (rawGitVersion) => {
    const gitVersion = rawGitVersion.split(' ')[2];
    const gitMinorVersion = gitVersion.split('.')[1];
    return parseInt(gitMinorVersion);
  };

  cloneSpecificFolder = () => {
    // Safeguard in case the tmp is not deleted and already have a remote defined
    this.removeTmpFolder();
    this.createTmpFolder();
    exec(`git --version`, (error, stdout, stderr) => {
      if (error) {
        console.error(chalk`{red exec error: ${error}}`);
        return;
      }
      const cloneImplementation =
        this.getGitMinorVersion(stdout) < 25 ? SparseCheckoutGitV217 : SparseCheckoutGitV225;

      new cloneImplementation(this.resourcesData).execute();
      this.moveFromTmpFolderToIconFolder();
      this.removeTmpFolder();
    });
  };

  cloneRepo = () => {
    execSync(
      `git clone "${this.resourcesData.repoUrl}" --depth=1 --single-branch --branch ${this.resourcesData.branch} ${AssetsPath} -o repo`,
      {
        stdio: 'inherit', // we need this so node will print the command output
      },
    );
  };

  removeTmpFolder = () => {
    exec(`[ -d "${TempDir}" ] && rm -rf ${TempDir}`);
  };

  createTmpFolder = () => {
    exec(`[ -d "${TempDir}" ] && rm -rf ${TempDir}`);
    exec(`mkdir -p ${TempDir}`);
  };

  moveFromTmpFolderToIconFolder = () => {
    exec(`cp -r ${TempDir}/${this.resourcesData.folder}/* ${AssetsPath}`, {
      stdio: 'inherit',
      cwd: ProjectDir,
    });
  };

  copyFromLocalDirectory = () => {
    exec(`cp -r ${this.resourcesData.path}* ${AssetsPath}`, (error, stdout, stderr) => {
      if (error) {
        console.error(chalk`{red exec error: ${error}}`);
        return;
      }

      if (stderr) {
        console.error(`stderr: ${stderr}`);
        return;
      }

      console.log(stdout);
      console.log(
        chalk`{green The icons have been successfully obtained. Check "${path.relative(
          ProjectDir,
          AssetsPath,
        )}".}`,
      );
    });
  };
}

const getIcons = (resourcesData) => {
  const iconManager = new IconManager(resourcesData);

  if (resourcesData.hasOwnProperty('repoUrl') || resourcesData.hasOwnProperty('path')) {
    exec(`[ -d "${AssetsPath}" ] && rm -rf ${AssetsPath}`);
  }
  exec(`mkdir -p ${AssetsPath}`);

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

module.exports = {
  getIcons,
};
