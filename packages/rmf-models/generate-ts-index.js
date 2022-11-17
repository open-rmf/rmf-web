const fs = require('fs');
const path = require('path');

function printHelp() {
  console.log(`usage: node ${path.basename(process.argv[1])} <dir>

This script creates an index.d.ts that exports all .d.ts files in the directory.
  `);
}

if (process.argv.length !== 3) {
  printHelp();
  process.exit(1);
}

const dir = process.argv[2];
const files = fs.readdirSync(dir).filter((f) => f.endsWith('.d.ts') && f !== 'index.d.ts');
const indexFile = fs.openSync(`${dir}/index.d.ts`, 'w');
files.forEach((f) => {
  fs.writeSync(indexFile, `export * from './${f.slice(0, f.length - 5)}';\n`);
});
