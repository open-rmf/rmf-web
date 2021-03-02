const jsonSchemaToTs = require('json-schema-to-typescript');
const path = require('path');
const fsp = require('fs').promises;

const files = process.argv.slice(2);
if (!files.length) {
  console.log(`usage: ${path.basename(process.argv[1])} <files>...`);
}

(async () => {
  for (const f of files) {
    const ts = await jsonSchemaToTs.compileFromFile(f);
    const baseName = path.basename(f);
    const tsFileName = `${baseName.slice(0, baseName.lastIndexOf('.'))}.ts`;
    const tsFilePath = path.join(__dirname, 'lib', 'models', tsFileName);
    fsp.writeFile(tsFilePath, ts);
    console.log(tsFilePath);
  }
})();
