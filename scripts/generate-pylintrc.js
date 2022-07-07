/**
 * Unlike eslint, pylint does not support inheriting of configs. It does however support generating
 * config based on command line args, which can include the --rcfile args, this allows us to
 * do pseudo inheritance by generating the .pylintrc files with this script.
 */
const { execSync } = require('child_process');
const fs = require('fs');

fs.copyFileSync(`${__dirname}/base.pylintrc`, `${__dirname}/../packages/api-server/.pylintrc`);
fs.copyFileSync(`${__dirname}/base.pylintrc`, `${__dirname}/../packages/ros-translator/.pylintrc`);
let result = execSync(
  `pipenv run pylint --rcfile=${__dirname}/base.pylintrc --ignore=CVS,test --generate-rcfile`,
  {
    stdio: ['inherit', 'pipe', 'inherit'],
  },
);
fs.writeFileSync(
  `${__dirname}/../packages/ros-translator/ros_translator/typescript/.pylintrc`,
  result,
);
