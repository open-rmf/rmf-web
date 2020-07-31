const chalk = require('chalk');
const inquirer = require('inquirer');
var fs = require('fs');

if (fs.existsSync('./.assets.json')) {
  console.log(chalk`
  {blue You already configured an icon source!} 👋`);
  return;
}

console.log(chalk`
Hello! 👋

{bold.red IMPORTANT} 🚨 You are about to configure where you are going to bring the icons for the Romi Dashboard project `);
console.log(chalk`
You have two options:
{cyan →} 1. {blue wget} the assets from a remote source.
{cyan →} 2. {blue copy} assets from a specific folder.
`);

inquirer
  .prompt([
    {
      name: 'WGET_OR_COPY',
      message: chalk`{blue What option are you going to choose?}`,
      validate: input => /^[1-2-]$/.test(input) || 'Invalid option',
    },
    {
      name: 'URL',
      message: 'Set URL',
      when: keys => keys['WGET_OR_COPY'] === '1',
      validate: input =>
        /[(http(s)?):\/\/(www\.)?a-zA-Z0-9@:%._\+~#=]{2,256}\.[a-z]{2,6}\b([-a-zA-Z0-9@:%_\+.~#?&//=]*)/.test(
          input,
        ) || 'Not a valid URL',
    },
    {
      name: 'COPY',
      message: 'Set absolute PATH',
      when: keys => keys['WGET_OR_COPY'] === '2',
      validate: input => /^(\/[^\/]+){0,20}\/?$/gm.test(input) || 'Not a valid PATH',
    },
  ])
  .then(keys => {
    let information;
    if (keys.WGET_OR_COPY === '1') {
      information = { url: keys.URL };
    }
    if (keys.WGET_OR_COPY === '2') {
      information = { path: keys.COPY };
    }
    fs.writeFile('.assets.json', JSON.stringify(information), function(err) {
      if (err) throw err;
      console.log('File .assets.json was created successfully.');
    });
  })
  .catch(error => console.error(error));
