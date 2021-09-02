const { readdirSync, readFileSync } = require('fs');
const { resolve } = require('path');

const artifactsDir = process.argv[2];
readdirSync(artifactsDir).forEach((fileName) => {
  const contents = JSON.parse(readFileSync(resolve(artifactsDir, fileName)));
  console.log(`::notice title=(${contents.browserName}) ${contents.suite}::${contents.publicUrl}`);
});
