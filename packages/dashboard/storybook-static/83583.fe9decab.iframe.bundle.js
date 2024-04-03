'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [83583],
  {
    '../../node_modules/.pnpm/@mdx-js+react@3.0.1_@types+react@18.2.14_react@18.2.0/node_modules/@mdx-js/react/index.js':
      (__unused_webpack_module, __webpack_exports__, __webpack_require__) => {
        __webpack_require__.r(__webpack_exports__),
          __webpack_require__.d(__webpack_exports__, {
            MDXProvider: () => MDXProvider,
            useMDXComponents: () => useMDXComponents,
          });
        var react = __webpack_require__(
          '../../node_modules/.pnpm/react@18.2.0/node_modules/react/index.js',
        );
        const emptyComponents = {},
          MDXContext = react.createContext(emptyComponents);
        function useMDXComponents(components) {
          const contextComponents = react.useContext(MDXContext);
          return react.useMemo(
            function () {
              return 'function' == typeof components
                ? components(contextComponents)
                : { ...contextComponents, ...components };
            },
            [contextComponents, components],
          );
        }
        function MDXProvider(properties) {
          let allComponents;
          return (
            (allComponents = properties.disableParentContext
              ? 'function' == typeof properties.components
                ? properties.components(emptyComponents)
                : properties.components || emptyComponents
              : useMDXComponents(properties.components)),
            react.createElement(MDXContext.Provider, { value: allComponents }, properties.children)
          );
        }
      },
  },
]);
