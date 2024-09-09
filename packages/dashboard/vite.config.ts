/// <reference types="vitest" />

import react from '@vitejs/plugin-react-swc';
import { defineConfig, Plugin } from 'vite';

import appConfig from './app-config.json';

/**
 * The goal of this plugin is to inject global variables to `index.html`, allowing
 * a crude way to configure some variables after the bundle is built via `sed`.
 *
 * An example use case is building a dockerfile, because the domain in a dev or staging
 * environment is typically different from prod, we would normally end up needing to build
 * different images (which is really detrimental for staging). In such scenario, you can
 * set the `rmfServerUrl` to a placeholder like `__RMF_SERVER_URL_PLACEHOLDER__` in
 * `app-config.json`, then do a search and replace at the container entrypoint.
 *
 * The reason for doing this outside the app code is to avoid these variables from
 * being modified by the bundler, and to reduce the chance that the search and replace modify
 * unintended code.
 */
const injectGlobals: Plugin = {
  name: 'injectRuntimeArgs',
  transformIndexHtml: {
    order: 'pre', // must be injected before the app
    handler: () => {
      return [
        {
          tag: 'script',
          injectTo: 'head',
          children: `const APP_CONFIG=${JSON.stringify(appConfig)}`,
        },
      ];
    },
  },
};

function booleanToString(b: boolean | null | undefined) {
  return b ? 'true' : 'false';
}

const buildConfig = appConfig.buildConfig;

// https://vitejs.dev/config/
export default defineConfig({
  base: buildConfig.baseUrl,
  define: {
    APP_CONFIG_AUTH_PROVIDER: `'${buildConfig.authProvider}'`,
    APP_CONFIG_ENABLE_CUSTOM_TABS: `${booleanToString(buildConfig.customTabs)}`,
    APP_CONFIG_ENABLE_ADMIN_TAB: `${booleanToString(buildConfig.adminTab)}`,
  },
  plugins: [injectGlobals, react()],
  test: {
    environment: 'jsdom',
    globals: true,
    passWithNoTests: true,
  },
});
