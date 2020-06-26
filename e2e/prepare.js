const fs = require('fs');

// generates soss config based on the current directory, we need to do this because soss
// requires an absolute path for the cert and key.
let sossConfig = fs.readFileSync(`${__dirname}/soss.yaml.in`, { encoding: 'utf8' });
sossConfig = sossConfig.replace(/{{pwd}}/g, __dirname);
fs.writeFileSync(`${__dirname}/soss.yaml`, sossConfig);