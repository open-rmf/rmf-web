const { execSync } = require('child_process');
const fs = require('fs');
const path = require('path');

const jsonSchemaToTs = require('json-schema-to-typescript');

const rmfMsgs = process.argv.slice(2);

console.log('generate schemas:');
execSync(`pipenv run python ${__dirname}/generate-schemas.py`, { stdio: 'inherit' });

const schemas = fs.readdirSync('build/schema/').map((f) => `build/schema/${f}`);

const tortoiseDir = path.join(__dirname, 'lib', 'tortoise');
fs.rmdirSync(tortoiseDir, { recursive: true, force: true });
fs.mkdirSync(tortoiseDir, { recursive: true });

console.log('generate models:');
(async () => {
  for (const f of schemas) {
    const ts = await jsonSchemaToTs.compileFromFile(f);
    const baseName = path.basename(f);
    const tsFileName = `${baseName.slice(0, baseName.lastIndexOf('.'))}.ts`;
    const tsFilePath = path.join(tortoiseDir, tsFileName);
    fs.writeFileSync(tsFilePath, ts);
    console.log(tsFilePath);
  }
  fs.writeFileSync(
    path.join(tortoiseDir, 'GENERATED'),
    'THIS DIRECTORY IS GENERATED, DO NOT EDIT!!',
  );

  execSync(`pipenv run python -m ros_translator -t=typescript -o=lib/ros ${rmfMsgs.join(' ')}`, {
    stdio: 'inherit',
  });
  fs.writeFileSync(
    path.join(__dirname, 'lib', 'ros', 'GENERATED'),
    'THIS DIRECTORY IS GENERATED, DO NOT EDIT!!',
  );
  execSync(`../../node_modules/.bin/prettier -w lib/ros lib/tortoise`, { stdio: 'inherit' });
})();
