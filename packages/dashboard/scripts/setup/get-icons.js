'use strict';

// FIXME: do not use shell commands to support cross platform.

const { execSync } = require('child_process');
const chalk = require('chalk');
const path = require('path');
const fs = require('fs');

const ProjectDir = __dirname.slice(0, __dirname.length - '/scripts/setup'.length);
const ResourcesPath = `${ProjectDir}/src/assets/resources`;
const TempDir = `${ProjectDir}/tmp`;

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
      throw new Error('Cannot find AMENT_PREFIX_PATH');
    }
    return process.env.AMENT_PREFIX_PATH;
  }
}

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
      `git clone "${this.resourcesData.repoUrl}" --no-checkout  --depth=1 --single-branch --branch "${this.resourcesData.branch}" "${TempDir}" -o repo`,
      {
        stdio: 'inherit',
      },
    );

    execSync(
      `git sparse-checkout init --cone &&
            git config core.sparseCheckout 1 &&
            git sparse-checkout set "${this.resourcesData.folder}" &&
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
      `git config core.sparseCheckout true && echo "${this.resourcesData.folder}" >> .git/info/sparse-checkout`,
      {
        stdio: 'inherit',
        cwd: path.resolve(__dirname, TempDir),
      },
    );
    execSync(`git pull --depth=1 resources "${this.resourcesData.branch}"`, {
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
    const stdout = execSync(`git --version`).toString();
    const cloneImplementation =
      this.getGitMinorVersion(stdout) < 25 ? SparseCheckoutGitV217 : SparseCheckoutGitV225;

    new cloneImplementation(this.resourcesData).execute();
    this.moveFromTmpFolderToIconFolder();
    this.removeTmpFolder();
  };

  cloneRepo = () => {
    execSync(
      `git clone "${this.resourcesData.repoUrl}" --depth=1 --single-branch --branch ${this.resourcesData.branch} ${ResourcesPath} -o repo`,
      {
        stdio: 'inherit', // we need this so node will print the command output
      },
    );
  };

  removeTmpFolder = () => {
    try {
      execSync(`[ -d "${TempDir}" ] && rm -rf "${TempDir}"`);
    } catch {}
  };

  createTmpFolder = () => {
    try {
      execSync(`[ -d "${TempDir}" ] && rm -rf "${TempDir}"`);
    } catch {}
    execSync(`mkdir -p "${TempDir}"`);
  };

  moveFromTmpFolderToIconFolder = () => {
    execSync(`cp -r "${TempDir}/${this.resourcesData.folder}"/* "${ResourcesPath}/"`, {
      stdio: 'inherit',
      cwd: ProjectDir,
    });
  };

  _copyDir = (from, to) => {
    if (!from.endsWith('/')) from += '/';
    const stdout = execSync(`cp -r "${from}"* "${to}"`).toString();
    console.log(stdout);
    console.log(
      chalk`{green The icons have been successfully obtained. Check "${path.relative(
        ProjectDir,
        ResourcesPath,
      )}".}`,
    );
  };

  copyFromLocalDirectory = () => {
    this._copyDir(this.resourcesData.path, ResourcesPath);
  };

  copyFromRosPackage = () => {
    const ament = new Ament();
    const packageDir = ament.findPackage(this.resourcesData.rosPackage);

    this._copyDir(`${packageDir}${this.resourcesData.path}`, ResourcesPath);
  };
}

const getIcons = (resourcesData) => {
  const iconManager = new IconManager(resourcesData);

  try {
    execSync(`[ -d "${ResourcesPath}" ] && rm -rf "${ResourcesPath}"`);
  } catch {}
  execSync(`mkdir -p "${ResourcesPath}"`);

  if (resourcesData.hasOwnProperty('repoUrl')) {
    // If we don't want to clone a specific folder of the repo, it'll clone the whole repo
    if (!resourcesData.folder) {
      iconManager.cloneRepo();
      return;
    }
    iconManager.cloneSpecificFolder();
  } else if (resourcesData.hasOwnProperty('rosPackage')) {
    iconManager.copyFromRosPackage();
  } else {
    iconManager.copyFromLocalDirectory();
  }
};

module.exports = {
  getIcons,
};
