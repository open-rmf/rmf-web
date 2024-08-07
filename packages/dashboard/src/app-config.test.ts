/**
 * @vitest-environment node
 */

import childProcess from 'child_process';
import fs from 'fs';
import { expect, it } from 'vitest';

it('app-config json schema is up to date', () => {
  const curSchema = fs.readFileSync('app-config.schema.json');
  const newSchema = childProcess.execSync(
    'pnpm typescript-json-schema tsconfig.gen.json AppConfig --required',
  );
  if (!newSchema.equals(curSchema)) {
    expect.fail(
      'app config schema has changed, please run `pnpm gen-app-config-schema` to update it',
    );
  }
});
