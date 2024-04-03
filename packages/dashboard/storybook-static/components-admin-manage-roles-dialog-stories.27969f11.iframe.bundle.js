'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [43880],
  {
    '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/Security.js':
      (__unused_webpack_module, exports, __webpack_require__) => {
        var _interopRequireDefault = __webpack_require__(
          '../../node_modules/.pnpm/@babel+runtime@7.18.6/node_modules/@babel/runtime/helpers/interopRequireDefault.js',
        );
        exports.A = void 0;
        var _createSvgIcon = _interopRequireDefault(
            __webpack_require__(
              '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/utils/createSvgIcon.js',
            ),
          ),
          _jsxRuntime = __webpack_require__(
            '../../node_modules/.pnpm/react@18.2.0/node_modules/react/jsx-runtime.js',
          ),
          _default = (0, _createSvgIcon.default)(
            (0, _jsxRuntime.jsx)('path', {
              d: 'M12 1 3 5v6c0 5.55 3.84 10.74 9 12 5.16-1.26 9-6.45 9-12V5l-9-4zm0 10.99h7c-.53 4.12-3.28 7.79-7 8.94V12H5V6.3l7-3.11v8.8z',
            }),
            'Security',
          );
        exports.A = _default;
      },
    './src/components/admin/manage-roles-dialog.stories.tsx': (
      __unused_webpack_module,
      __webpack_exports__,
      __webpack_require__,
    ) => {
      __webpack_require__.r(__webpack_exports__),
        __webpack_require__.d(__webpack_exports__, {
          Default: () => Default,
          __namedExportsOrder: () => __namedExportsOrder,
          default: () => manage_roles_dialog_stories,
        });
      var react = __webpack_require__(
          '../../node_modules/.pnpm/react@18.2.0/node_modules/react/index.js',
        ),
        styled = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/styles/styled.js',
        ),
        Card = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Card/Card.js',
        ),
        Dialog = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Dialog/Dialog.js',
        ),
        DialogTitle = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/DialogTitle/DialogTitle.js',
        ),
        DialogContent = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/DialogContent/DialogContent.js',
        ),
        DialogActions = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/DialogActions/DialogActions.js',
        ),
        Button = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Button/Button.js',
        ),
        CardHeader = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/CardHeader/CardHeader.js',
        ),
        Divider = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Divider/Divider.js',
        ),
        List = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/List/List.js',
        ),
        ListItem = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/ListItem/ListItem.js',
        ),
        ListItemText = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/ListItemText/ListItemText.js',
        ),
        Security = __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/Security.js',
        ),
        dist = __webpack_require__('../react-components/dist/index.js'),
        app_contexts = __webpack_require__('./src/components/app-contexts.tsx'),
        jsx_runtime = __webpack_require__(
          '../../node_modules/.pnpm/react@18.2.0/node_modules/react/jsx-runtime.js',
        );
      const prefix = 'manage-roles-dialog',
        classes = {
          action: ''.concat(prefix, '-action'),
          list: ''.concat(prefix, '-list'),
          dialogContent: ''.concat(prefix, '-content'),
          dialogButton: ''.concat(prefix, '-button'),
        },
        StyledCard = (0, styled.Ay)((props) => (0, jsx_runtime.jsx)(Card.A, { ...props }))(
          (_ref) => {
            let { theme } = _ref;
            return {
              ['& .'.concat(classes.action)]: { margin: 0 },
              ['& .'.concat(classes.list)]: {
                paddingLeft: theme.spacing(1),
                paddingRight: theme.spacing(1),
              },
              ['& .'.concat(classes.dialogContent)]: { height: '50vh' },
              ['& .'.concat(classes.dialogButton)]: { width: 100 },
            };
          },
        );
      function ManageRolesDialog(_ref2) {
        let { defaultAssignedRoles, setOpen, getAllRoles, saveRoles, open, ...dialogProps } = _ref2;
        const safeAsync = (0, dist.Yb)(),
          [availableRoles, setAvailableRoles] = react.useState([]),
          [assignedRoles, setAssignedRoles] = react.useState([]),
          [loading, setLoading] = react.useState(!1),
          [saving, setSaving] = react.useState(!1),
          { showAlert } = react.useContext(app_contexts.J3);
        react.useEffect(() => {
          open &&
            getAllRoles &&
            (setLoading(!0),
            (async () => {
              try {
                const allRoles = await safeAsync(getAllRoles());
                setAvailableRoles(
                  allRoles.filter((r) => -1 === defaultAssignedRoles.indexOf(r)).sort(),
                ),
                  setAssignedRoles(defaultAssignedRoles.sort());
              } catch (e) {
                showAlert('error', 'Failed to get roles: '.concat(e.message));
              } finally {
                setLoading(!1);
              }
            })());
        }, [open, getAllRoles, defaultAssignedRoles, showAlert, safeAsync]);
        return (0, jsx_runtime.jsxs)(Dialog.A, {
          maxWidth: 'md',
          fullWidth: !0,
          open,
          onClose: () => setOpen && setOpen(!1),
          ...dialogProps,
          children: [
            (0, jsx_runtime.jsx)(DialogTitle.A, { children: 'Manage Roles' }),
            (0, jsx_runtime.jsx)(DialogContent.A, {
              dividers: !0,
              className: classes.dialogContent,
              children: (0, jsx_runtime.jsx)(dist.Rh, {
                loading,
                size: '5em',
                children: (0, jsx_runtime.jsx)(dist.Mq, {
                  leftItems: availableRoles,
                  rightItems: assignedRoles,
                  leftTitle: 'Available Roles',
                  rightTitle: 'Assigned Roles',
                  onTransfer: (left, right) => {
                    setAvailableRoles(left.sort()), setAssignedRoles(right.sort());
                  },
                }),
              }),
            }),
            (0, jsx_runtime.jsxs)(DialogActions.A, {
              children: [
                (0, jsx_runtime.jsx)(Button.A, {
                  variant: 'outlined',
                  color: 'secondary',
                  'aria-label': 'Cancel',
                  className: classes.dialogButton,
                  onClick: () => {
                    setLoading(!1), setOpen && setOpen(!1);
                  },
                  disabled: saving,
                  children: 'Cancel',
                }),
                (0, jsx_runtime.jsx)(Button.A, {
                  variant: 'contained',
                  color: 'primary',
                  'aria-label': 'OK',
                  disabled: saving,
                  className: classes.dialogButton,
                  onClick: () => {
                    setSaving(!0),
                      (async () => {
                        try {
                          saveRoles && (await safeAsync(saveRoles(assignedRoles))),
                            setSaving(!1),
                            setOpen && setOpen(!1);
                        } catch (e) {
                          setSaving(!1),
                            showAlert('error', 'Failed to save roles: '.concat(e.message));
                        }
                      })();
                  },
                  children: (0, jsx_runtime.jsx)(dist.Rh, {
                    hideChildren: !0,
                    loading: saving,
                    size: '1.5em',
                    color: 'inherit',
                    children: 'Save',
                  }),
                }),
              ],
            }),
          ],
        });
      }
      function ManageRolesCard(_ref3) {
        let { assignedRoles, getAllRoles, saveRoles, ...otherProps } = _ref3;
        const [openDialog, setOpenDialog] = react.useState(!1);
        return (0, jsx_runtime.jsxs)(StyledCard, {
          variant: 'outlined',
          ...otherProps,
          children: [
            (0, jsx_runtime.jsx)(CardHeader.A, {
              title: 'Roles',
              titleTypographyProps: { variant: 'h5' },
              avatar: (0, jsx_runtime.jsx)(Security.A, {}),
              action: (0, jsx_runtime.jsx)(Button.A, {
                variant: 'contained',
                color: 'primary',
                'aria-label': 'Add/Remove',
                onClick: () => {
                  setOpenDialog(!0);
                },
                children: 'Add/Remove',
              }),
              classes: { action: classes.action },
            }),
            (0, jsx_runtime.jsx)(Divider.A, {}),
            (0, jsx_runtime.jsx)(List.A, {
              dense: !0,
              className: classes.list,
              children: assignedRoles.map((r) =>
                (0, jsx_runtime.jsx)(
                  ListItem.Ay,
                  { children: (0, jsx_runtime.jsx)(ListItemText.A, { children: r }) },
                  r,
                ),
              ),
            }),
            (0, jsx_runtime.jsx)(ManageRolesDialog, {
              open: openDialog,
              setOpen: setOpenDialog,
              defaultAssignedRoles: assignedRoles,
              getAllRoles,
              saveRoles,
            }),
          ],
        });
      }
      (ManageRolesDialog.__docgenInfo = {
        description: '',
        methods: [],
        displayName: 'ManageRolesDialog',
        props: {
          defaultAssignedRoles: {
            required: !0,
            tsType: { name: 'Array', elements: [{ name: 'string' }], raw: 'string[]' },
            description: '',
          },
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
          getAllRoles: {
            required: !1,
            tsType: {
              name: 'signature',
              type: 'function',
              raw: '() => Promise<string[]>',
              signature: {
                arguments: [],
                return: {
                  name: 'Promise',
                  elements: [{ name: 'Array', elements: [{ name: 'string' }], raw: 'string[]' }],
                  raw: 'Promise<string[]>',
                },
              },
            },
            description: '',
          },
          saveRoles: {
            required: !1,
            tsType: {
              name: 'signature',
              type: 'function',
              raw: '(roles: string[]) => Promise<void>',
              signature: {
                arguments: [
                  {
                    type: { name: 'Array', elements: [{ name: 'string' }], raw: 'string[]' },
                    name: 'roles',
                  },
                ],
                return: { name: 'Promise', elements: [{ name: 'void' }], raw: 'Promise<void>' },
              },
            },
            description: '',
          },
        },
        composes: ['Omit'],
      }),
        (ManageRolesCard.__docgenInfo = {
          description: '',
          methods: [],
          displayName: 'ManageRolesCard',
          props: {
            assignedRoles: {
              required: !0,
              tsType: { name: 'Array', elements: [{ name: 'string' }], raw: 'string[]' },
              description: '',
            },
          },
          composes: ['CardProps', 'Pick'],
        });
      const manage_roles_dialog_stories = { title: 'Admin/Manage Roles Card' },
        allRoles = [];
      for (let i = 0; i < 5; i++) allRoles.push('role'.concat(i));
      const Default = (args) =>
        (0, jsx_runtime.jsx)(ManageRolesCard, {
          assignedRoles: ['role1'],
          getAllRoles: async () => (await new Promise((res) => setTimeout(res, 100)), allRoles),
          saveRoles: async () => {
            await new Promise((res) => setTimeout(res, 100));
          },
          ...args,
        });
      (Default.storyName = 'Manage Roles Card'),
        (Default.parameters = {
          ...Default.parameters,
          docs: {
            ...Default.parameters?.docs,
            source: {
              originalSource:
                "args => {\n  return <ManageRolesCard assignedRoles={['role1']} getAllRoles={async () => {\n    await new Promise(res => setTimeout(res, 100));\n    return allRoles;\n  }} saveRoles={async () => {\n    await new Promise(res => setTimeout(res, 100));\n  }} {...args} />;\n}",
              ...Default.parameters?.docs?.source,
            },
          },
        });
      const __namedExportsOrder = ['Default'];
    },
  },
]);
