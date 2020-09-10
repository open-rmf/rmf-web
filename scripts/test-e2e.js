const concurrently = require('concurrently');
const { execSync } = require('child_process');
const fs = require('fs');

const resources = `{
  "repoUrl": "https://github.com/osrf/rmf_demos.git",
  "folder": "rmf_dashboard_resources/office/",
  "branch": "master"
}
`;

fs.writeFile('.resources.json', resources, function(err) {
  if (err) {
    throw err;
  }
  console.log(`File .resources.json was created successfully.`);
  execSync('node ./scripts/setup/get-icons.js');

  execSync('npm run build:e2e', { stdio: 'inherit' });

  // wrap in double quotes to support args with spaces
  const wdioArgs = process.argv
    .slice(2)
    .map(arg => `"${arg}"`)
    .join(' ');

  concurrently(['npm:start:auth', 'npm:start:react:e2e', `npm:test:e2e:wdio -- ${wdioArgs}`], {
    killOthers: ['success', 'failure'],
    successCondition: 'first',
  }).catch(e => {
    console.error(e);
    process.exitCode = -1;
  });
});
