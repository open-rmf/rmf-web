const concurrently = require('concurrently');
const { services } = require('./config');

// wrap in double quotes to support args with spaces
const wdioArgs = process.argv
  .slice(2)
  .map((arg) => `"${arg}"`)
  .join(' ');

concurrently([...services, `wdio ${wdioArgs}`], {
  killOthers: ['success', 'failure'],
  successCondition: 'first',
  prefix: 'none',
}).catch((e) => {
  console.error(e);
  process.exitCode = -1;
});
