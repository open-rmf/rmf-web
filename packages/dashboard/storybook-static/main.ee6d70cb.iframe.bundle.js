(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [38792],
  {
    '../../node_modules/.pnpm/@storybook+instrumenter@8.0.5/node_modules/@storybook/instrumenter/dist sync recursive':
      (module) => {
        function webpackEmptyContext(req) {
          var e = new Error("Cannot find module '" + req + "'");
          throw ((e.code = 'MODULE_NOT_FOUND'), e);
        }
        (webpackEmptyContext.keys = () => []),
          (webpackEmptyContext.resolve = webpackEmptyContext),
          (webpackEmptyContext.id =
            '../../node_modules/.pnpm/@storybook+instrumenter@8.0.5/node_modules/@storybook/instrumenter/dist sync recursive'),
          (module.exports = webpackEmptyContext);
      },
    './storybook-config-entry.js': (
      __unused_webpack_module,
      __unused_webpack___webpack_exports__,
      __webpack_require__,
    ) => {
      'use strict';
      var external_STORYBOOK_MODULE_GLOBAL_ = __webpack_require__('@storybook/global'),
        external_STORYBOOK_MODULE_PREVIEW_API_ = __webpack_require__('@storybook/preview-api'),
        external_STORYBOOK_MODULE_CHANNELS_ = __webpack_require__('@storybook/channels');
      const importers = [
        async (path) => {
          if (
            !/^\.[\\/](?:src(?:\/(?!\.)(?:(?:(?!(?:^|\/)\.).)*?)\/|\/|$)(?!\.)(?=.)[^/]*?\.mdx)$/.exec(
              path,
            )
          )
            return;
          const pathRemainder = path.substring(6);
          return __webpack_require__(
            './src lazy recursive ^\\.\\/.*$ include: (?%21.*node_modules)(?:\\/src(?:\\/(?%21\\.)(?:(?:(?%21(?:^%7C\\/)\\.).)*?)\\/%7C\\/%7C$)(?%21\\.)(?=.)[^/]*?\\.mdx)$',
          )('./' + pathRemainder);
        },
        async (path) => {
          if (
            !/^\.[\\/](?:src(?:\/(?!\.)(?:(?:(?!(?:^|\/)\.).)*?)\/|\/|$)(?!\.)(?=.)[^/]*?\.stories\.(js|jsx|mjs|ts|tsx))$/.exec(
              path,
            )
          )
            return;
          const pathRemainder = path.substring(6);
          return __webpack_require__(
            './src lazy recursive ^\\.\\/.*$ include: (?%21.*node_modules)(?:\\/src(?:\\/(?%21\\.)(?:(?:(?%21(?:^%7C\\/)\\.).)*?)\\/%7C\\/%7C$)(?%21\\.)(?=.)[^/]*?\\.stories\\.(js%7Cjsx%7Cmjs%7Cts%7Ctsx))$',
          )('./' + pathRemainder);
        },
      ];
      const channel = (0, external_STORYBOOK_MODULE_CHANNELS_.createBrowserChannel)({
        page: 'preview',
      });
      external_STORYBOOK_MODULE_PREVIEW_API_.addons.setChannel(channel),
        'DEVELOPMENT' === external_STORYBOOK_MODULE_GLOBAL_.global.CONFIG_TYPE &&
          (window.__STORYBOOK_SERVER_CHANNEL__ = channel);
      const preview = new external_STORYBOOK_MODULE_PREVIEW_API_.PreviewWeb(
        async function importFn(path) {
          for (let i = 0; i < importers.length; i++) {
            const moduleExports = await ((x = () => importers[i](path)), x());
            if (moduleExports) return moduleExports;
          }
          var x;
        },
        () =>
          (0, external_STORYBOOK_MODULE_PREVIEW_API_.composeConfigs)([
            __webpack_require__(
              '../../node_modules/.pnpm/@storybook+react@8.0.5_react-dom@18.2.0_react@18.2.0_typescript@5.4.3/node_modules/@storybook/react/dist/entry-preview.mjs',
            ),
            __webpack_require__(
              '../../node_modules/.pnpm/@storybook+react@8.0.5_react-dom@18.2.0_react@18.2.0_typescript@5.4.3/node_modules/@storybook/react/dist/entry-preview-docs.mjs',
            ),
            __webpack_require__(
              '../../node_modules/.pnpm/@storybook+addon-links@8.0.5_react@18.2.0/node_modules/@storybook/addon-links/dist/preview.js',
            ),
            __webpack_require__(
              '../../node_modules/.pnpm/@storybook+addon-essentials@8.0.5_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@storybook/addon-essentials/dist/docs/preview.js',
            ),
            __webpack_require__(
              '../../node_modules/.pnpm/@storybook+addon-essentials@8.0.5_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@storybook/addon-essentials/dist/actions/preview.js',
            ),
            __webpack_require__(
              '../../node_modules/.pnpm/@storybook+addon-essentials@8.0.5_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@storybook/addon-essentials/dist/backgrounds/preview.js',
            ),
            __webpack_require__(
              '../../node_modules/.pnpm/@storybook+addon-essentials@8.0.5_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@storybook/addon-essentials/dist/viewport/preview.js',
            ),
            __webpack_require__(
              '../../node_modules/.pnpm/@storybook+addon-essentials@8.0.5_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@storybook/addon-essentials/dist/measure/preview.js',
            ),
            __webpack_require__(
              '../../node_modules/.pnpm/@storybook+addon-essentials@8.0.5_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@storybook/addon-essentials/dist/outline/preview.js',
            ),
            __webpack_require__(
              '../../node_modules/.pnpm/@storybook+addon-essentials@8.0.5_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@storybook/addon-essentials/dist/highlight/preview.js',
            ),
            __webpack_require__(
              '../../node_modules/.pnpm/@storybook+addon-interactions@8.0.5_@types+jest@29.5.12_jest@29.7.0/node_modules/@storybook/addon-interactions/dist/preview.js',
            ),
            __webpack_require__('./.storybook/preview.ts'),
          ]),
      );
      (window.__STORYBOOK_PREVIEW__ = preview),
        (window.__STORYBOOK_STORY_STORE__ = preview.storyStore),
        (window.__STORYBOOK_ADDONS_CHANNEL__ = channel);
    },
    '../../node_modules/.pnpm/@storybook+test@8.0.5_@types+jest@29.5.12_jest@29.7.0/node_modules/@storybook/test/dist sync recursive':
      (module) => {
        function webpackEmptyContext(req) {
          var e = new Error("Cannot find module '" + req + "'");
          throw ((e.code = 'MODULE_NOT_FOUND'), e);
        }
        (webpackEmptyContext.keys = () => []),
          (webpackEmptyContext.resolve = webpackEmptyContext),
          (webpackEmptyContext.id =
            '../../node_modules/.pnpm/@storybook+test@8.0.5_@types+jest@29.5.12_jest@29.7.0/node_modules/@storybook/test/dist sync recursive'),
          (module.exports = webpackEmptyContext);
      },
    './.storybook/preview.ts': (
      __unused_webpack_module,
      __webpack_exports__,
      __webpack_require__,
    ) => {
      'use strict';
      __webpack_require__.r(__webpack_exports__),
        __webpack_require__.d(__webpack_exports__, { default: () => __WEBPACK_DEFAULT_EXPORT__ });
      const __WEBPACK_DEFAULT_EXPORT__ = {
        parameters: { controls: { matchers: { color: /(background|color)$/i, date: /Date$/i } } },
      };
    },
    './src lazy recursive ^\\.\\/.*$ include: (?%21.*node_modules)(?:\\/src(?:\\/(?%21\\.)(?:(?:(?%21(?:^%7C\\/)\\.).)*?)\\/%7C\\/%7C$)(?%21\\.)(?=.)[^/]*?\\.mdx)$':
      (module) => {
        function webpackEmptyAsyncContext(req) {
          return Promise.resolve().then(() => {
            var e = new Error("Cannot find module '" + req + "'");
            throw ((e.code = 'MODULE_NOT_FOUND'), e);
          });
        }
        (webpackEmptyAsyncContext.keys = () => []),
          (webpackEmptyAsyncContext.resolve = webpackEmptyAsyncContext),
          (webpackEmptyAsyncContext.id =
            './src lazy recursive ^\\.\\/.*$ include: (?%21.*node_modules)(?:\\/src(?:\\/(?%21\\.)(?:(?:(?%21(?:^%7C\\/)\\.).)*?)\\/%7C\\/%7C$)(?%21\\.)(?=.)[^/]*?\\.mdx)$'),
          (module.exports = webpackEmptyAsyncContext);
      },
    './src lazy recursive ^\\.\\/.*$ include: (?%21.*node_modules)(?:\\/src(?:\\/(?%21\\.)(?:(?:(?%21(?:^%7C\\/)\\.).)*?)\\/%7C\\/%7C$)(?%21\\.)(?=.)[^/]*?\\.stories\\.(js%7Cjsx%7Cmjs%7Cts%7Ctsx))$':
      (module, __unused_webpack_exports, __webpack_require__) => {
        var map = {
          './components/admin/add-permission-dialog.stories': [
            './src/components/admin/add-permission-dialog.stories.tsx',
            29282,
            80506,
            36994,
            81167,
            90846,
          ],
          './components/admin/add-permission-dialog.stories.tsx': [
            './src/components/admin/add-permission-dialog.stories.tsx',
            29282,
            80506,
            36994,
            81167,
            90846,
          ],
          './components/admin/create-role-dialog.stories': [
            './src/components/admin/create-role-dialog.stories.tsx',
            29282,
            80506,
            36994,
            81167,
            74536,
          ],
          './components/admin/create-role-dialog.stories.tsx': [
            './src/components/admin/create-role-dialog.stories.tsx',
            29282,
            80506,
            36994,
            81167,
            74536,
          ],
          './components/admin/create-user-dialog.stories': [
            './src/components/admin/create-user-dialog.stories.tsx',
            29282,
            80506,
            36994,
            81167,
            44803,
          ],
          './components/admin/create-user-dialog.stories.tsx': [
            './src/components/admin/create-user-dialog.stories.tsx',
            29282,
            80506,
            36994,
            81167,
            44803,
          ],
          './components/admin/drawer.stories': [
            './src/components/admin/drawer.stories.tsx',
            29282,
            96213,
            50741,
          ],
          './components/admin/drawer.stories.tsx': [
            './src/components/admin/drawer.stories.tsx',
            29282,
            96213,
            50741,
          ],
          './components/admin/manage-roles-dialog.stories': [
            './src/components/admin/manage-roles-dialog.stories.tsx',
            29282,
            80506,
            36994,
            81167,
            43880,
          ],
          './components/admin/manage-roles-dialog.stories.tsx': [
            './src/components/admin/manage-roles-dialog.stories.tsx',
            29282,
            80506,
            36994,
            81167,
            43880,
          ],
          './components/admin/permissions-card.stories': [
            './src/components/admin/permissions-card.stories.tsx',
            29282,
            80506,
            36994,
            81167,
            33673,
          ],
          './components/admin/permissions-card.stories.tsx': [
            './src/components/admin/permissions-card.stories.tsx',
            29282,
            80506,
            36994,
            81167,
            33673,
          ],
          './components/admin/role-list-card.stories': [
            './src/components/admin/role-list-card.stories.tsx',
            29282,
            80506,
            36994,
            29190,
            81167,
            62118,
          ],
          './components/admin/role-list-card.stories.tsx': [
            './src/components/admin/role-list-card.stories.tsx',
            29282,
            80506,
            36994,
            29190,
            81167,
            62118,
          ],
          './components/admin/user-list-card.stories': [
            './src/components/admin/user-list-card.stories.tsx',
            29282,
            80506,
            36994,
            96213,
            81167,
            71999,
          ],
          './components/admin/user-list-card.stories.tsx': [
            './src/components/admin/user-list-card.stories.tsx',
            29282,
            80506,
            36994,
            96213,
            81167,
            71999,
          ],
          './components/admin/user-profile.stories': [
            './src/components/admin/user-profile.stories.tsx',
            29282,
            80506,
            36994,
            81167,
            93443,
          ],
          './components/admin/user-profile.stories.tsx': [
            './src/components/admin/user-profile.stories.tsx',
            29282,
            80506,
            36994,
            81167,
            93443,
          ],
        };
        function webpackAsyncContext(req) {
          if (!__webpack_require__.o(map, req))
            return Promise.resolve().then(() => {
              var e = new Error("Cannot find module '" + req + "'");
              throw ((e.code = 'MODULE_NOT_FOUND'), e);
            });
          var ids = map[req],
            id = ids[0];
          return Promise.all(ids.slice(1).map(__webpack_require__.e)).then(() =>
            __webpack_require__(id),
          );
        }
        (webpackAsyncContext.keys = () => Object.keys(map)),
          (webpackAsyncContext.id =
            './src lazy recursive ^\\.\\/.*$ include: (?%21.*node_modules)(?:\\/src(?:\\/(?%21\\.)(?:(?:(?%21(?:^%7C\\/)\\.).)*?)\\/%7C\\/%7C$)(?%21\\.)(?=.)[^/]*?\\.stories\\.(js%7Cjsx%7Cmjs%7Cts%7Ctsx))$'),
          (module.exports = webpackAsyncContext);
      },
    '@storybook/channels': (module) => {
      'use strict';
      module.exports = __STORYBOOK_MODULE_CHANNELS__;
    },
    '@storybook/client-logger': (module) => {
      'use strict';
      module.exports = __STORYBOOK_MODULE_CLIENT_LOGGER__;
    },
    '@storybook/core-events/preview-errors': (module) => {
      'use strict';
      module.exports = __STORYBOOK_MODULE_CORE_EVENTS_PREVIEW_ERRORS__;
    },
    '@storybook/core-events': (module) => {
      'use strict';
      module.exports = __STORYBOOK_MODULE_CORE_EVENTS__;
    },
    '@storybook/global': (module) => {
      'use strict';
      module.exports = __STORYBOOK_MODULE_GLOBAL__;
    },
    '@storybook/preview-api': (module) => {
      'use strict';
      module.exports = __STORYBOOK_MODULE_PREVIEW_API__;
    },
  },
  (__webpack_require__) => {
    __webpack_require__.O(0, [43109], () => {
      return (
        (moduleId = './storybook-config-entry.js'),
        __webpack_require__((__webpack_require__.s = moduleId))
      );
      var moduleId;
    });
    __webpack_require__.O();
  },
]);
