'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [62118],
  {
    './src/components/admin/role-list-card.stories.tsx': (
      __unused_webpack_module,
      __webpack_exports__,
      __webpack_require__,
    ) => {
      __webpack_require__.r(__webpack_exports__),
        __webpack_require__.d(__webpack_exports__, {
          Default: () => Default,
          __namedExportsOrder: () => __namedExportsOrder,
          default: () => role_list_card_stories,
        });
      var styled = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/styles/styled.js',
        ),
        Card = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Card/Card.js',
        ),
        Accordion = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Accordion/Accordion.js',
        ),
        AccordionSummary = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/AccordionSummary/AccordionSummary.js',
        ),
        Typography = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Typography/Typography.js',
        ),
        AccordionDetails = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/AccordionDetails/AccordionDetails.js',
        ),
        Grid = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Grid/Grid.js',
        ),
        Button = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Button/Button.js',
        ),
        CardHeader = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/CardHeader/CardHeader.js',
        ),
        IconButton = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/IconButton/IconButton.js',
        ),
        Divider = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Divider/Divider.js',
        ),
        AddCircle = __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/AddCircle.js',
        ),
        Delete = __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/Delete.js',
        ),
        ExpandMore = __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/ExpandMore.js',
        ),
        Security = __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/Security.js',
        ),
        react = __webpack_require__(
          '../../node_modules/.pnpm/react@18.2.0/node_modules/react/index.js',
        ),
        dist = __webpack_require__('../react-components/dist/index.js'),
        app_contexts = __webpack_require__('./src/components/app-contexts.tsx'),
        create_role_dialog = __webpack_require__('./src/components/admin/create-role-dialog.tsx'),
        permissions_card = __webpack_require__('./src/components/admin/permissions-card.tsx'),
        jsx_runtime = __webpack_require__(
          '../../node_modules/.pnpm/react@18.2.0/node_modules/react/jsx-runtime.js',
        );
      const classes = {
          permissionsCard: ''.concat('role-list-card', '-permissionscard'),
          deleteRoleButton: ''.concat('role-list-card', '-deleterolebutton'),
        },
        StyledCard = (0, styled.Ay)((props) => (0, jsx_runtime.jsx)(Card.A, { ...props }))(() => ({
          ['& .'.concat(classes.permissionsCard)]: { width: '100%' },
          ['& .'.concat(classes.deleteRoleButton)]: { float: 'right' },
        }));
      function RoleAccordion(_ref) {
        let { role, onDeleteClick, getPermissions, savePermission, removePermission } = _ref;
        return (0, jsx_runtime.jsxs)(Accordion.A, {
          TransitionProps: { unmountOnExit: !0 },
          children: [
            (0, jsx_runtime.jsx)(AccordionSummary.A, {
              expandIcon: (0, jsx_runtime.jsx)(ExpandMore.A, {}),
              children: (0, jsx_runtime.jsx)(Typography.A, { children: role }),
            }),
            (0, jsx_runtime.jsx)(AccordionDetails.A, {
              children: (0, jsx_runtime.jsxs)(Grid.Ay, {
                container: !0,
                direction: 'column',
                wrap: 'nowrap',
                children: [
                  (0, jsx_runtime.jsx)(Grid.Ay, {
                    item: !0,
                    children: (0, jsx_runtime.jsx)(Button.A, {
                      variant: 'contained',
                      color: 'secondary',
                      startIcon: (0, jsx_runtime.jsx)(Delete.A, {}),
                      className: classes.deleteRoleButton,
                      onClick: onDeleteClick,
                      children: 'Delete Role',
                    }),
                  }),
                  (0, jsx_runtime.jsx)(Grid.Ay, {
                    item: !0,
                    children: (0, jsx_runtime.jsx)(permissions_card.f, {
                      className: classes.permissionsCard,
                      getPermissions,
                      savePermission,
                      removePermission,
                    }),
                  }),
                ],
              }),
            }),
          ],
        });
      }
      function RoleListCard(_ref2) {
        let { getRoles, deleteRole, getPermissions, savePermission, removePermission, createRole } =
          _ref2;
        const safeAsync = (0, dist.Yb)(),
          [roles, setRoles] = react.useState([]),
          [loading, setLoading] = react.useState(!0),
          [openDialog, setOpenDialog] = react.useState(!1),
          [selectedDeleteRole, setSelectedDeleteRole] = react.useState(null),
          [deleting, setDeleting] = react.useState(!1),
          { showAlert } = react.useContext(app_contexts.J3),
          refresh = react.useCallback(async () => {
            if (getRoles) {
              setLoading(!0);
              try {
                const newRoles = await safeAsync(getRoles());
                setRoles(newRoles.sort());
              } catch (e) {
                showAlert('error', 'Failed to get roles: '.concat(e.message));
              } finally {
                setLoading(!1);
              }
            }
          }, [getRoles, showAlert, safeAsync]);
        react.useEffect(() => {
          refresh();
        }, [refresh]);
        const getRolePermissions = react.useMemo(
            () => roles.map((r) => getPermissions && (() => getPermissions(r))),
            [roles, getPermissions],
          ),
          saveRolePermissions = react.useMemo(
            () => roles.map((r) => savePermission && ((p) => savePermission(r, p))),
            [roles, savePermission],
          ),
          removeRolePermissions = react.useMemo(
            () => roles.map((r) => removePermission && ((p) => removePermission(r, p))),
            [roles, removePermission],
          ),
          handleRoleDelete = react.useMemo(
            () =>
              roles.map((r) => () => {
                setDeleting(!1), setSelectedDeleteRole(r);
              }),
            [roles],
          );
        return (0, jsx_runtime.jsxs)(StyledCard, {
          variant: 'outlined',
          children: [
            (0, jsx_runtime.jsx)(CardHeader.A, {
              title: 'Roles',
              titleTypographyProps: { variant: 'h5' },
              avatar: (0, jsx_runtime.jsx)(Security.A, {}),
              action: (0, jsx_runtime.jsx)(IconButton.A, {
                onClick: () => setOpenDialog(!0),
                'aria-label': 'create role',
                children: (0, jsx_runtime.jsx)(AddCircle.A, { fontSize: 'large' }),
              }),
            }),
            (0, jsx_runtime.jsx)(Divider.A, {}),
            (0, jsx_runtime.jsxs)(dist.Rh, {
              loading,
              size: '50px',
              children: [
                roles.map((r, i) =>
                  (0, jsx_runtime.jsx)(
                    RoleAccordion,
                    {
                      role: r,
                      getPermissions: getRolePermissions[i],
                      savePermission: saveRolePermissions[i],
                      removePermission: removeRolePermissions[i],
                      onDeleteClick: handleRoleDelete[i],
                    },
                    r,
                  ),
                ),
                0 === roles.length && (0, jsx_runtime.jsx)('div', { style: { height: 100 } }),
              ],
            }),
            openDialog &&
              (0, jsx_runtime.jsx)(create_role_dialog.B, {
                open: openDialog,
                setOpen: setOpenDialog,
                createRole:
                  createRole &&
                  (async (r) => {
                    await createRole(r), refresh();
                  }),
              }),
            selectedDeleteRole &&
              (0, jsx_runtime.jsx)(dist.KK, {
                open: !!selectedDeleteRole,
                title: 'Confirm Delete',
                submitting: !!deleting,
                onClose: () => setSelectedDeleteRole(null),
                onSubmit: async () => {
                  try {
                    setDeleting(!0),
                      deleteRole && (await safeAsync(deleteRole(selectedDeleteRole))),
                      refresh();
                  } catch (e) {
                    showAlert('error', 'Failed to delete user: '.concat(e.message));
                  } finally {
                    setSelectedDeleteRole(null);
                  }
                },
                children: (0, jsx_runtime.jsx)(Typography.A, {
                  children: 'Are you sure you want to delete "'.concat(selectedDeleteRole, '"?'),
                }),
              }),
          ],
        });
      }
      RoleListCard.__docgenInfo = {
        description: '',
        methods: [],
        displayName: 'RoleListCard',
        props: {
          getRoles: {
            required: !1,
            tsType: {
              name: 'signature',
              type: 'function',
              raw: '() => Promise<string[]> | string[]',
              signature: {
                arguments: [],
                return: {
                  name: 'union',
                  raw: 'Promise<string[]> | string[]',
                  elements: [
                    {
                      name: 'Promise',
                      elements: [
                        { name: 'Array', elements: [{ name: 'string' }], raw: 'string[]' },
                      ],
                      raw: 'Promise<string[]>',
                    },
                    { name: 'Array', elements: [{ name: 'string' }], raw: 'string[]' },
                  ],
                },
              },
            },
            description: '',
          },
          deleteRole: {
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
          getPermissions: {
            required: !1,
            tsType: {
              name: 'signature',
              type: 'function',
              raw: '(role: string) => Promise<Permission[]> | Permission[]',
              signature: {
                arguments: [{ type: { name: 'string' }, name: 'role' }],
                return: {
                  name: 'union',
                  raw: 'Promise<Permission[]> | Permission[]',
                  elements: [
                    {
                      name: 'Promise',
                      elements: [
                        { name: 'Array', elements: [{ name: 'Permission' }], raw: 'Permission[]' },
                      ],
                      raw: 'Promise<Permission[]>',
                    },
                    { name: 'Array', elements: [{ name: 'Permission' }], raw: 'Permission[]' },
                  ],
                },
              },
            },
            description: '',
          },
          savePermission: {
            required: !1,
            tsType: {
              name: 'signature',
              type: 'function',
              raw: '(role: string, permission: Permission) => Promise<void> | void',
              signature: {
                arguments: [
                  { type: { name: 'string' }, name: 'role' },
                  { type: { name: 'Permission' }, name: 'permission' },
                ],
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
          removePermission: {
            required: !1,
            tsType: {
              name: 'signature',
              type: 'function',
              raw: '(role: string, permission: Permission) => Promise<void> | void',
              signature: {
                arguments: [
                  { type: { name: 'string' }, name: 'role' },
                  { type: { name: 'Permission' }, name: 'permission' },
                ],
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
        composes: ['Pick'],
      };
      var permissions = __webpack_require__('./src/components/permissions.ts');
      const role_list_card_stories = { title: 'Admin/Role List Card', component: RoleListCard },
        Default = (args) =>
          (0, jsx_runtime.jsx)(RoleListCard, {
            ...args,
            getRoles: async () => (
              await new Promise((res) => setTimeout(res, 100)), ['role4', 'role2', 'role3', 'role1']
            ),
            getPermissions: async () => (
              await new Promise((res) => setTimeout(res, 100)),
              [
                { action: permissions.q0.TaskCancel, authz_grp: 'group1' },
                { action: permissions.q0.TaskRead, authz_grp: 'group1' },
              ]
            ),
            createRole: () => new Promise((res) => setTimeout(res, 100)),
            deleteRole: () => new Promise((res) => setTimeout(res, 100)),
          });
      (Default.storyName = 'Role List Card'),
        (Default.parameters = {
          ...Default.parameters,
          docs: {
            ...Default.parameters?.docs,
            source: {
              originalSource:
                "args => {\n  return <RoleListCard {...args} getRoles={async () => {\n    await new Promise(res => setTimeout(res, 100));\n    return ['role4', 'role2', 'role3', 'role1'];\n  }} getPermissions={async () => {\n    await new Promise(res => setTimeout(res, 100));\n    return [{\n      action: RmfAction.TaskCancel,\n      authz_grp: 'group1'\n    }, {\n      action: RmfAction.TaskRead,\n      authz_grp: 'group1'\n    }];\n  }} createRole={() => new Promise(res => setTimeout(res, 100))} deleteRole={() => new Promise(res => setTimeout(res, 100))} />;\n}",
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
    './src/components/admin/permissions-card.tsx': (
      __unused_webpack_module,
      __webpack_exports__,
      __webpack_require__,
    ) => {
      __webpack_require__.d(__webpack_exports__, { f: () => PermissionsCard });
      var _mui_material__WEBPACK_IMPORTED_MODULE_7__ = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Paper/Paper.js',
        ),
        _mui_material__WEBPACK_IMPORTED_MODULE_8__ = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Toolbar/Toolbar.js',
        ),
        _mui_material__WEBPACK_IMPORTED_MODULE_9__ = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Typography/Typography.js',
        ),
        _mui_material__WEBPACK_IMPORTED_MODULE_10__ = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/IconButton/IconButton.js',
        ),
        _mui_material__WEBPACK_IMPORTED_MODULE_12__ = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/TableContainer/TableContainer.js',
        ),
        _mui_material__WEBPACK_IMPORTED_MODULE_13__ = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Table/Table.js',
        ),
        _mui_material__WEBPACK_IMPORTED_MODULE_14__ = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/TableHead/TableHead.js',
        ),
        _mui_material__WEBPACK_IMPORTED_MODULE_15__ = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/TableRow/TableRow.js',
        ),
        _mui_material__WEBPACK_IMPORTED_MODULE_16__ = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/TableCell/TableCell.js',
        ),
        _mui_material__WEBPACK_IMPORTED_MODULE_17__ = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/TableBody/TableBody.js',
        ),
        _mui_material__WEBPACK_IMPORTED_MODULE_18__ = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Button/Button.js',
        ),
        _mui_icons_material_AddCircle__WEBPACK_IMPORTED_MODULE_11__ = __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/AddCircle.js',
        ),
        _mui_icons_material_Delete__WEBPACK_IMPORTED_MODULE_19__ = __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/Delete.js',
        ),
        react__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(
          '../../node_modules/.pnpm/react@18.2.0/node_modules/react/index.js',
        ),
        _mui_material__WEBPACK_IMPORTED_MODULE_6__ = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/styles/styled.js',
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
        _add_permission_dialog__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(
          './src/components/admin/add-permission-dialog.tsx',
        ),
        react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__ = __webpack_require__(
          '../../node_modules/.pnpm/react@18.2.0/node_modules/react/jsx-runtime.js',
        );
      const prefix = 'permissions-card',
        classes = {
          title: ''.concat(prefix, '-title'),
          tableContainer: ''.concat(prefix, '-table-container'),
          controlsButton: ''.concat(prefix, '-controls-button'),
        },
        StyledPaper = (0, _mui_material__WEBPACK_IMPORTED_MODULE_6__.Ay)((props) =>
          (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsx)(
            _mui_material__WEBPACK_IMPORTED_MODULE_7__.A,
            { ...props },
          ),
        )((_ref) => {
          let { theme } = _ref;
          return {
            ['& .'.concat(classes.title)]: { flex: '1 1 100%' },
            ['& .'.concat(classes.tableContainer)]: {
              marginLeft: theme.spacing(4),
              marginRight: theme.spacing(4),
              width: 'auto',
            },
            ['& .'.concat(classes.controlsButton)]: { float: 'right' },
          };
        });
      function PermissionsCard(_ref2) {
        let { getPermissions, savePermission, removePermission, ...otherProps } = _ref2;
        const safeAsync = (0, react_components__WEBPACK_IMPORTED_MODULE_1__.Yb)(),
          [loading, setLoading] = react__WEBPACK_IMPORTED_MODULE_0__.useState(!1),
          [permissions, setPermissions] = react__WEBPACK_IMPORTED_MODULE_0__.useState([]),
          [openDialog, setOpenDialog] = react__WEBPACK_IMPORTED_MODULE_0__.useState(!1),
          { showAlert } = react__WEBPACK_IMPORTED_MODULE_0__.useContext(
            _app_contexts__WEBPACK_IMPORTED_MODULE_2__.J3,
          ),
          refresh = react__WEBPACK_IMPORTED_MODULE_0__.useCallback(async () => {
            if (getPermissions) {
              setLoading(!0);
              try {
                const newPermissions = await safeAsync(getPermissions());
                newPermissions.sort((a, b) =>
                  a.action < b.action
                    ? -1
                    : a.action > b.action
                      ? 1
                      : a.authz_grp < b.authz_grp
                        ? -1
                        : a.authz_grp > b.authz_grp
                          ? 1
                          : 0,
                ),
                  setPermissions(newPermissions);
              } catch (e) {
                showAlert('error', 'Failed to get permissions: '.concat(e.message));
              } finally {
                setLoading(!1);
              }
            }
          }, [getPermissions, showAlert, safeAsync]);
        return (
          react__WEBPACK_IMPORTED_MODULE_0__.useEffect(() => {
            refresh();
          }, [refresh]),
          (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsxs)(StyledPaper, {
            elevation: 0,
            ...otherProps,
            children: [
              (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsxs)(
                _mui_material__WEBPACK_IMPORTED_MODULE_8__.A,
                {
                  children: [
                    (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsx)(
                      _mui_material__WEBPACK_IMPORTED_MODULE_9__.A,
                      { variant: 'h6', className: classes.title, children: 'Permissions' },
                    ),
                    (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsx)(
                      _mui_material__WEBPACK_IMPORTED_MODULE_10__.A,
                      {
                        onClick: () => setOpenDialog(!0),
                        'aria-label': 'add permission',
                        children: (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsx)(
                          _mui_icons_material_AddCircle__WEBPACK_IMPORTED_MODULE_11__.A,
                          { fontSize: 'large' },
                        ),
                      },
                    ),
                  ],
                },
              ),
              (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsx)(
                react_components__WEBPACK_IMPORTED_MODULE_1__.Rh,
                {
                  loading,
                  children: (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsx)(
                    _mui_material__WEBPACK_IMPORTED_MODULE_12__.A,
                    {
                      id: 'permission-table',
                      className: classes.tableContainer,
                      children: (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsxs)(
                        _mui_material__WEBPACK_IMPORTED_MODULE_13__.A,
                        {
                          children: [
                            (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsx)(
                              _mui_material__WEBPACK_IMPORTED_MODULE_14__.A,
                              {
                                children: (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsxs)(
                                  _mui_material__WEBPACK_IMPORTED_MODULE_15__.A,
                                  {
                                    children: [
                                      (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsx)(
                                        _mui_material__WEBPACK_IMPORTED_MODULE_16__.A,
                                        { children: 'Action' },
                                      ),
                                      (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsx)(
                                        _mui_material__WEBPACK_IMPORTED_MODULE_16__.A,
                                        { children: 'Authorization Group' },
                                      ),
                                      (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsx)(
                                        _mui_material__WEBPACK_IMPORTED_MODULE_16__.A,
                                        {},
                                      ),
                                    ],
                                  },
                                ),
                              },
                            ),
                            (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsx)(
                              _mui_material__WEBPACK_IMPORTED_MODULE_17__.A,
                              {
                                children: permissions.map((p, idx) =>
                                  (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsxs)(
                                    _mui_material__WEBPACK_IMPORTED_MODULE_15__.A,
                                    {
                                      children: [
                                        (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsx)(
                                          _mui_material__WEBPACK_IMPORTED_MODULE_16__.A,
                                          {
                                            children: (0,
                                            _permissions__WEBPACK_IMPORTED_MODULE_3__.Mc)(p.action),
                                          },
                                        ),
                                        (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsx)(
                                          _mui_material__WEBPACK_IMPORTED_MODULE_16__.A,
                                          { children: p.authz_grp },
                                        ),
                                        (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsx)(
                                          _mui_material__WEBPACK_IMPORTED_MODULE_16__.A,
                                          {
                                            children: (0,
                                            react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsx)(
                                              _mui_material__WEBPACK_IMPORTED_MODULE_18__.A,
                                              {
                                                variant: 'contained',
                                                color: 'secondary',
                                                startIcon: (0,
                                                react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsx)(
                                                  _mui_icons_material_Delete__WEBPACK_IMPORTED_MODULE_19__.A,
                                                  {},
                                                ),
                                                className: classes.controlsButton,
                                                onClick:
                                                  removePermission &&
                                                  (async () => {
                                                    try {
                                                      await removePermission(p), refresh();
                                                    } catch (e) {
                                                      showAlert(
                                                        'error',
                                                        'Failed to remove permission: '.concat(
                                                          e.message,
                                                        ),
                                                      );
                                                    }
                                                  }),
                                                children: 'Remove',
                                              },
                                            ),
                                          },
                                        ),
                                      ],
                                    },
                                    idx,
                                  ),
                                ),
                              },
                            ),
                          ],
                        },
                      ),
                    },
                  ),
                },
              ),
              openDialog &&
                (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_5__.jsx)(
                  _add_permission_dialog__WEBPACK_IMPORTED_MODULE_4__.b,
                  {
                    open: openDialog,
                    setOpen: setOpenDialog,
                    savePermission:
                      savePermission &&
                      (async (p) => {
                        await savePermission(p), refresh();
                      }),
                  },
                ),
            ],
          })
        );
      }
      PermissionsCard.__docgenInfo = {
        description: '',
        methods: [],
        displayName: 'PermissionsCard',
        props: {
          getPermissions: {
            required: !1,
            tsType: {
              name: 'signature',
              type: 'function',
              raw: '() => Promise<Permission[]> | Permission[]',
              signature: {
                arguments: [],
                return: {
                  name: 'union',
                  raw: 'Promise<Permission[]> | Permission[]',
                  elements: [
                    {
                      name: 'Promise',
                      elements: [
                        { name: 'Array', elements: [{ name: 'Permission' }], raw: 'Permission[]' },
                      ],
                      raw: 'Promise<Permission[]>',
                    },
                    { name: 'Array', elements: [{ name: 'Permission' }], raw: 'Permission[]' },
                  ],
                },
              },
            },
            description: '',
          },
          removePermission: {
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
        composes: ['PaperProps', 'Pick'],
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
