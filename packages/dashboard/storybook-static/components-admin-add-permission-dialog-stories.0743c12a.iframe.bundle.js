'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [90846],
  {
    './src/components/admin/add-permission-dialog.stories.tsx': (
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
      var _components_admin_add_permission_dialog__WEBPACK_IMPORTED_MODULE_0__ =
          __webpack_require__('./src/components/admin/add-permission-dialog.tsx'),
        react_jsx_runtime__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(
          '../../node_modules/.pnpm/react@18.2.0/node_modules/react/jsx-runtime.js',
        );
      const __WEBPACK_DEFAULT_EXPORT__ = {
          title: 'Admin/Add Permission Dialog',
          component: _components_admin_add_permission_dialog__WEBPACK_IMPORTED_MODULE_0__.b,
        },
        Default = (args) =>
          (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_1__.jsx)(
            _components_admin_add_permission_dialog__WEBPACK_IMPORTED_MODULE_0__.b,
            {
              ...args,
              open: !0,
              setOpen: () => {},
              savePermission: () => new Promise((res) => setTimeout(res, 100)),
            },
          );
      (Default.storyName = 'Add Permission Dialog'),
        (Default.parameters = {
          ...Default.parameters,
          docs: {
            ...Default.parameters?.docs,
            source: {
              originalSource:
                'args => {\n  return <AddPermissionDialog {...args} open={true} setOpen={() => {}} savePermission={() => new Promise(res => setTimeout(res, 100))} />;\n}',
              ...Default.parameters?.docs?.source,
            },
          },
        });
      const __namedExportsOrder = ['Default'];
    },
    './src/components/admin/add-permission-dialog.tsx': (
      __unused_webpack_module,
      __webpack_exports__,
      __webpack_require__,
    ) => {
      __webpack_require__.d(__webpack_exports__, { b: () => AddPermissionDialog });
      var _mui_material__WEBPACK_IMPORTED_MODULE_5__ = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/TextField/TextField.js',
        ),
        _mui_material__WEBPACK_IMPORTED_MODULE_6__ = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/MenuItem/MenuItem.js',
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
        _permissions__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(
          './src/components/permissions.ts',
        ),
        react_jsx_runtime__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(
          '../../node_modules/.pnpm/react@18.2.0/node_modules/react/jsx-runtime.js',
        );
      function AddPermissionDialog(_ref) {
        let { open, setOpen, savePermission } = _ref;
        const safeAsync = (0, react_components__WEBPACK_IMPORTED_MODULE_1__.Yb)(),
          [action, setAction] = react__WEBPACK_IMPORTED_MODULE_0__.useState(''),
          [authzGrp, setAuthzGrp] = react__WEBPACK_IMPORTED_MODULE_0__.useState(''),
          [actionError, setActionError] = react__WEBPACK_IMPORTED_MODULE_0__.useState(!1),
          [authzGrpError, setAuthzGrpError] = react__WEBPACK_IMPORTED_MODULE_0__.useState(!1),
          [saving, setSaving] = react__WEBPACK_IMPORTED_MODULE_0__.useState(!1),
          { showAlert } = react__WEBPACK_IMPORTED_MODULE_0__.useContext(
            _app_contexts__WEBPACK_IMPORTED_MODULE_2__.J3,
          );
        return (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_4__.jsxs)(
          react_components__WEBPACK_IMPORTED_MODULE_1__.KK,
          {
            open,
            title: 'Add Permission',
            confirmText: 'Save',
            submitting: saving,
            onSubmit: async () => {
              if (
                (() => {
                  let error = !1;
                  return (
                    action ? setActionError(!1) : (setActionError(!0), (error = !0)),
                    authzGrp ? setAuthzGrpError(!1) : (setAuthzGrpError(!0), (error = !0)),
                    !error
                  );
                })()
              ) {
                setSaving(!0);
                try {
                  savePermission &&
                    (await safeAsync(savePermission({ action, authz_grp: authzGrp }))),
                    setSaving(!1),
                    setOpen && setOpen(!1);
                } catch (e) {
                  setSaving(!1),
                    showAlert('error', 'Failed to save permission: '.concat(e.message));
                }
              }
            },
            onClose: () => setOpen && setOpen(!1),
            children: [
              (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_4__.jsx)(
                _mui_material__WEBPACK_IMPORTED_MODULE_5__.A,
                {
                  id: 'action-input',
                  select: !0,
                  variant: 'outlined',
                  fullWidth: !0,
                  autoFocus: !0,
                  margin: 'normal',
                  label: 'Action',
                  value: action,
                  onChange: (ev) => setAction(ev.target.value),
                  error: actionError,
                  helperText: 'Required',
                  children: Object.values(_permissions__WEBPACK_IMPORTED_MODULE_3__.q0).map((act) =>
                    (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_4__.jsx)(
                      _mui_material__WEBPACK_IMPORTED_MODULE_6__.A,
                      {
                        value: act,
                        children: (0, _permissions__WEBPACK_IMPORTED_MODULE_3__.Mc)(act),
                      },
                      act,
                    ),
                  ),
                },
              ),
              (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_4__.jsx)(
                _mui_material__WEBPACK_IMPORTED_MODULE_5__.A,
                {
                  id: 'authz-grp-input',
                  variant: 'outlined',
                  fullWidth: !0,
                  label: 'Authorization Group',
                  value: authzGrp,
                  onChange: (ev) => setAuthzGrp(ev.target.value),
                  error: authzGrpError,
                  helperText: 'Required',
                },
              ),
            ],
          },
        );
      }
      AddPermissionDialog.__docgenInfo = {
        description: '',
        methods: [],
        displayName: 'AddPermissionDialog',
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
          savePermission: {
            required: !1,
            tsType: {
              name: 'signature',
              type: 'function',
              raw: '(permission: Permission) => Promise<void> | void',
              signature: {
                arguments: [{ type: { name: 'Permission' }, name: 'permission' }],
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
    './src/components/permissions.ts': (
      __unused_webpack_module,
      __webpack_exports__,
      __webpack_require__,
    ) => {
      __webpack_require__.d(__webpack_exports__, { Mc: () => getActionText, q0: () => RmfAction });
      let RmfAction = (function (RmfAction) {
        return (
          (RmfAction.TaskRead = 'task_read'),
          (RmfAction.TaskSubmit = 'task_submit'),
          (RmfAction.TaskCancel = 'task_cancel'),
          RmfAction
        );
      })({});
      function getActionText(action) {
        return action.replace(/(?:^|_)([a-z])/g, (_, p1) => ' '.concat(p1.toUpperCase())).slice(1);
      }
    },
  },
]);
