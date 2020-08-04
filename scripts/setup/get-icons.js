'use strict';
const { exec } = require('child_process');
const fs = require('fs');
const chalk = require('chalk');

let rawdata = fs.readFileSync('./.assets.json');
let data = JSON.parse(rawdata);
if (data.hasOwnProperty('url')) {
  exec('[ -d "public/assets/icons/" ] && rm -rf public/assets/icons');
  exec('mkdir -p public/assets/icons');
  exec(`svn checkout ${data.url} public/assets/icons/`);
  console.log(chalk`{green The icons have been successfully obtained. Check public/assets/icons/}`);
} else if (data.hasOwnProperty('path')) {
  exec('[ -d "public/assets/icons/" ] && rm -rf public/assets/icons');
  exec('mkdir -p public/assets/icons');
  exec(`cp -r ${data.path}* public/assets/icons/`);
  console.log(chalk`{green The icons have been successfully obtained. Check public/assets/icons/}`);
}
