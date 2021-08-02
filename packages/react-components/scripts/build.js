const { execSync } = require('child_process');
const { copyFileSync } = require('fs');

execSync('tsc --build', { stdio: 'inherit' });
copyFileSync('package.json', 'dist/package.json');
