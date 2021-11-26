# RMF TOOLS

The RMF Tools package serves as a utility package for common helper tools used in the RMF interfaces. This package contains the following scripts:

### rmf-launcher

Helper function that launches and kills all required RMF processes. This assumes required rmf components are installed and launches them locally. An example on how to use:

```
require('rmf-tools').makeLauncher().launch();
```

### setup/create-config

A function to create a `.resource.json` file for fetching icons during setup.

### setup/geticons

A function to get icons from the above mentioned resource file.

Below is an example of how to use the 2 setup functions together:

```
const chalk = require('chalk');
const fs = require('fs');
const { getIcons, createConfigFile } = require('rmf-tools');

const ProjectDir = __dirname.slice(0, __dirname.length - '/scripts/setup'.length);

// check if config file/resource file exist
const resourcesPath = process.env.RMF_DASHBOARD_RESOURCES_FILE || `${ProjectDir}/.resources.json`;
const configExists = fs.existsSync(resourcesPath);

(async () => {
  // create config file if it does not exist
  if (configExists) {
    console.log(chalk`{blue Using existing resources config}`);
  } else {
    await createConfigFile();
  }

  // parse information from resource file
  const resourcesData = JSON.parse(fs.readFileSync(resourcesPath));

  // get icons from resourceData and write it to your package
  try {
    getIcons(resourcesData, <packageFolderName>);
  } catch (e) {
    console.error(e);
    process.exit(e.status);
  }
})();

```