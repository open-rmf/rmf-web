const chalk = require('chalk');
const inquirer = require('inquirer');
var fs = require('fs');

if (fs.existsSync('./.resources.json')) {
  console.log(chalk`
  {blue You already configured an icon source!} 👋`);
  return;
}

console.log(chalk`
Hello! 👋

{bold.red IMPORTANT} 🚨 You are about to configure where you are going to bring the icons for the Romi Dashboard project `);
console.log(chalk`
You have two options:
{cyan →} 1. {blue get} the assets from a remote GH source.
{cyan →} 2. {blue copy} assets from a specific folder.
`);

inquirer
  .prompt([
    {
      name: 'GET_OR_COPY',
      message: chalk`{blue What option are you going to choose?}`,
      validate: input => /^[1-2-]$/.test(input) || 'Invalid option',
    },
    {
      name: 'URL',
      // TODO: We should change the example once https://github.com/osrf/rmf_demos/pull/109 is merged.
      message:
        'Set URL. Example: https://github.com/matiasbavera/rmf_demos/branches/feat/createResourceFolder/rmf_dashboard_resources/office',
      when: keys => keys['GET_OR_COPY'] === '1',
      validate: input =>
        /[(http(s)?):\/\/(www\.)?a-zA-Z0-9@:%._\+~#=]{2,256}\.[a-z]{2,6}\b([-a-zA-Z0-9@:%_\+.~#?&//=]*)/.test(
          input,
        ) || 'Not a valid URL',
    },
    {
      name: 'COPY',
      message: 'Set absolute PATH',
      when: keys => keys['GET_OR_COPY'] === '2',
      validate: input => /^(\/[^\/]+){0,20}\/?$/gm.test(input) || 'Not a valid PATH',
    },
  ])
  .then(keys => {
    let information;
    if (keys.GET_OR_COPY === '1') {
      information = { url: keys.URL };
    }
    if (keys.GET_OR_COPY === '2') {
      information = { path: keys.COPY };
    }
    fs.writeFile('.resources.json', JSON.stringify(information), function(err) {
      if (err) throw err;
      console.log(chalk`{green File .resources.json was created successfully.}`);
    });
  })
  .catch(error => console.error(error));
