'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [44803],
  {
    './src/components/admin/create-user-dialog.stories.tsx': (
      __unused_webpack_module,
      __webpack_exports__,
      __webpack_require__,
    ) => {
      __webpack_require__.r(__webpack_exports__),
        __webpack_require__.d(__webpack_exports__, {
          Default: () => Default,
          __namedExportsOrder: () => __namedExportsOrder,
          default: () => __WEBPACK_DEFAULT_EXPORT__,
        });
      var _components_admin_create_user_dialog__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(
          './src/components/admin/create-user-dialog.tsx',
        ),
        react_jsx_runtime__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(
          '../../node_modules/.pnpm/react@18.2.0/node_modules/react/jsx-runtime.js',
        );
      const __WEBPACK_DEFAULT_EXPORT__ = {
          title: 'Admin/Create User Dialog',
          component: _components_admin_create_user_dialog__WEBPACK_IMPORTED_MODULE_0__._,
        },
        Default = (args) =>
          (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_1__.jsx)(
            _components_admin_create_user_dialog__WEBPACK_IMPORTED_MODULE_0__._,
            { ...args, open: !0, createUser: () => new Promise((res) => setTimeout(res, 100)) },
          );
      (Default.storyName = 'Create User Dialog'),
        (Default.parameters = {
          ...Default.parameters,
          docs: {
            ...Default.parameters?.docs,
            source: {
              originalSource:
                'args => {\n  return <CreateUserDialog {...args} open={true} createUser={() => new Promise(res => setTimeout(res, 100))} />;\n}',
              ...Default.parameters?.docs?.source,
            },
          },
        });
      const __namedExportsOrder = ['Default'];
    },
    './src/components/admin/create-user-dialog.tsx': (
      __unused_webpack_module,
      __webpack_exports__,
      __webpack_require__,
    ) => {
      __webpack_require__.d(__webpack_exports__, { _: () => CreateUserDialog });
      var _mui_material__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/TextField/TextField.js',
        ),
        react__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(
          '../../node_modules/.pnpm/react@18.2.0/node_modules/react/index.js',
        ),
        react_components__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(
          '../react-components/dist/index.js',
        ),
        _app_contexts__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(
          './src/components/app-contexts.tsx',
        ),
        react_jsx_runtime__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(
          '../../node_modules/.pnpm/react@18.2.0/node_modules/react/jsx-runtime.js',
        );
      function CreateUserDialog(_ref) {
        let { open, setOpen, createUser } = _ref;
        const safeAsync = (0, react_components__WEBPACK_IMPORTED_MODULE_1__.Yb)(),
          [creating, setCreating] = react__WEBPACK_IMPORTED_MODULE_0__.useState(!1),
          [username, setUsername] = react__WEBPACK_IMPORTED_MODULE_0__.useState(''),
          [usernameError, setUsernameError] = react__WEBPACK_IMPORTED_MODULE_0__.useState(!1),
          { showAlert } = react__WEBPACK_IMPORTED_MODULE_0__.useContext(
            _app_contexts__WEBPACK_IMPORTED_MODULE_2__.J3,
          );
        return (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_3__.jsx)(
          react_components__WEBPACK_IMPORTED_MODULE_1__.KK,
          {
            open,
            title: 'Create User',
            confirmText: 'Create',
            submitting: creating,
            onSubmit: async () => {
              if (
                (() => {
                  let error = !1;
                  return (
                    username ? setUsernameError(!1) : (setUsernameError(!0), (error = !0)), !error
                  );
                })()
              ) {
                setCreating(!0);
                try {
                  createUser && (await safeAsync(createUser(username))),
                    setCreating(!1),
                    setOpen && setOpen(!1);
                } catch (e) {
                  setCreating(!1), showAlert('error', 'Failed to create user: '.concat(e.message));
                }
              }
            },
            onClose: () => setOpen && setOpen(!1),
            children: (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_3__.jsx)(
              _mui_material__WEBPACK_IMPORTED_MODULE_4__.A,
              {
                id: 'username',
                variant: 'outlined',
                fullWidth: !0,
                autoFocus: !0,
                margin: 'normal',
                label: 'Username',
                value: username,
                onChange: (ev) => setUsername(ev.target.value),
                error: usernameError,
                helperText: 'Required',
              },
            ),
          },
        );
      }
      CreateUserDialog.__docgenInfo = {
        description: '',
        methods: [],
        displayName: 'CreateUserDialog',
        props: {
          open: { required: !0, tsType: { name: 'boolean' }, description: '' },
          setOpen: {
            required: !1,
            tsType: {
              name: 'signature',
              type: 'function',
              raw: '(open: boolean) => void',
              signature: {
                arguments: [{ type: { name: 'boolean' }, name: 'open' }],
                return: { name: 'void' },
              },
            },
            description: '',
          },
          createUser: {
            required: !1,
            tsType: {
              name: 'signature',
              type: 'function',
              raw: '(username: string) => Promise<void> | void',
              signature: {
                arguments: [{ type: { name: 'string' }, name: 'username' }],
                return: {
                  name: 'union',
                  raw: 'Promise<void> | void',
                  elements: [
                    { name: 'Promise', elements: [{ name: 'void' }], raw: 'Promise<void>' },
                    { name: 'void' },
                  ],
                },
              },
            },
            description: '',
          },
        },
      };
    },
  },
]);
