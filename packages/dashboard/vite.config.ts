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
      const globals = {
        RMF_SERVER_URL: appConfig.rmfServerUrl,
        TRAJECTORY_SERVER_URL: appConfig.trajectoryServerUrl,
      };
      return [
        {
          tag: 'script',
          injectTo: 'head',
          children: Object.entries(globals)
            .map(([k, v]) => `const ${k}='${v}'`)
            .join(';'),
        },
      ];
    },
  },
};

// https://vitejs.dev/config/
export default defineConfig({
  define: {
    APP_CONFIG_AUTH_PROVIDER: `'${appConfig.auth.provider}'`,
    APP_CONFIG_RMF_SERVER_URL: `'${appConfig.rmfServerUrl}'`,
  },
  plugins: [injectGlobals, react()],
  test: {
    environment: 'jsdom',
    globals: true,
  },
});
