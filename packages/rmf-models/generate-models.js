const { execSync } = require('child_process');
const fs = require('fs');
const path = require('path');

const rmfMsgs = process.argv.slice(2);

console.log('generate models:');
(async () => {
  execSync(`pipenv run python -m ros_translator -t=typescript -o=lib/ros ${rmfMsgs.join(' ')}`, {
    stdio: 'inherit',
  });
  fs.writeFileSync(
    path.join(__dirname, 'lib', 'ros', 'GENERATED'),
    'THIS DIRECTORY IS GENERATED, DO NOT EDIT!!',
  );
  execSync(`../../node_modules/.bin/prettier -w lib/ros`, { stdio: 'inherit' });
})();
