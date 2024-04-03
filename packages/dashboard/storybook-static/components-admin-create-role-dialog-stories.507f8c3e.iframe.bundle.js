'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [74536],
  {
    './src/components/admin/create-role-dialog.stories.tsx': (
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
      var _components_admin_create_role_dialog__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(
          './src/components/admin/create-role-dialog.tsx',
        ),
        react_jsx_runtime__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(
          '../../node_modules/.pnpm/react@18.2.0/node_modules/react/jsx-runtime.js',
        );
      const __WEBPACK_DEFAULT_EXPORT__ = {
          title: 'Admin/Create Role Dialog',
          component: _components_admin_create_role_dialog__WEBPACK_IMPORTED_MODULE_0__.B,
        },
        Default = (args) =>
          (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_1__.jsx)(
            _components_admin_create_role_dialog__WEBPACK_IMPORTED_MODULE_0__.B,
            { ...args, open: !0, createRole: () => new Promise((res) => setTimeout(res, 100)) },
          );
      (Default.storyName = 'Create Role Dialog'),
        (Default.parameters = {
          ...Default.parameters,
          docs: {
            ...Default.parameters?.docs,
            source: {
              originalSource:
                'args => {\n  return <CreateRoleDialog {...args} open={true} createRole={() => new Promise(res => setTimeout(res, 100))} />;\n}',
              ...Default.parameters?.docs?.source,
            },
          },
        });
      const __namedExportsOrder = ['Default'];
    },
    './src/components/admin/create-role-dialog.tsx': (
      __unused_webpack_module,
      __webpack_exports__,
      __webpack_require__,
    ) => {
      __webpack_require__.d(__webpack_exports__, { B: () => CreateRoleDialog });
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
      function CreateRoleDialog(_ref) {
        let { open, setOpen, createRole } = _ref;
        const safeAsync = (0, react_components__WEBPACK_IMPORTED_MODULE_1__.Yb)(),
          [creating, setCreating] = react__WEBPACK_IMPORTED_MODULE_0__.useState(!1),
          [role, setRole] = react__WEBPACK_IMPORTED_MODULE_0__.useState(''),
          [roleError, setRoleError] = react__WEBPACK_IMPORTED_MODULE_0__.useState(!1),
          { showAlert } = react__WEBPACK_IMPORTED_MODULE_0__.useContext(
            _app_contexts__WEBPACK_IMPORTED_MODULE_2__.J3,
          );
        return (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_3__.jsx)(
          react_components__WEBPACK_IMPORTED_MODULE_1__.KK,
          {
            open,
            title: 'Create Role',
            confirmText: 'Create',
            submitting: creating,
            onSubmit: async () => {
              if (
                (() => {
                  let error = !1;
                  return role ? setRoleError(!1) : (setRoleError(!0), (error = !0)), !error;
                })()
              ) {
                setCreating(!0);
                try {
                  createRole && (await safeAsync(createRole(role))),
                    setCreating(!1),
                    setOpen && setOpen(!1);
                } catch (e) {
                  setCreating(!1), showAlert('error', 'Failed to create role: '.concat(e.message));
                }
              }
            },
            onClose: () => setOpen && setOpen(!1),
            children: (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_3__.jsx)(
              _mui_material__WEBPACK_IMPORTED_MODULE_4__.A,
              {
                id: 'role',
                variant: 'outlined',
                fullWidth: !0,
                autoFocus: !0,
                label: 'Role',
                margin: 'normal',
                value: role,
                onChange: (ev) => setRole(ev.target.value),
                error: roleError,
                helperText: 'Required',
              },
            ),
          },
        );
      }
      CreateRoleDialog.__docgenInfo = {
        description: '',
        methods: [],
        displayName: 'CreateRoleDialog',
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
          createRole: {
            required: !1,
            tsType: {
              name: 'signature',
              type: 'function',
              raw: '(role: string) => Promise<void> | void',
              signature: {
                arguments: [{ type: { name: 'string' }, name: 'role' }],
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
