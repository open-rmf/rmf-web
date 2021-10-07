const { execSync } = require('child_process');
const { copyFileSync } = require('fs');

execSync('tsc --build ./tsconfig.build.json', { stdio: 'inherit' });
copyFileSync('package.json', 'dist/package.json');
