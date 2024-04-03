'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [33673],
  {
    '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/AddCircle.js':
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
              d: 'M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm5 11h-4v4h-2v-4H7v-2h4V7h2v4h4v2z',
            }),
            'AddCircle',
          );
        exports.A = _default;
      },
    '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/TableContainer/TableContainer.js':
      (__unused_webpack_module, __webpack_exports__, __webpack_require__) => {
        __webpack_require__.d(__webpack_exports__, { A: () => TableContainer_TableContainer });
        var esm_extends = __webpack_require__(
            '../../node_modules/.pnpm/@babel+runtime@7.18.6/node_modules/@babel/runtime/helpers/esm/extends.js',
          ),
          objectWithoutPropertiesLoose = __webpack_require__(
            '../../node_modules/.pnpm/@babel+runtime@7.18.6/node_modules/@babel/runtime/helpers/esm/objectWithoutPropertiesLoose.js',
          ),
          react = __webpack_require__(
            '../../node_modules/.pnpm/react@18.2.0/node_modules/react/index.js',
          ),
          clsx_m = __webpack_require__(
            '../../node_modules/.pnpm/clsx@1.2.1/node_modules/clsx/dist/clsx.m.js',
          ),
          composeClasses = __webpack_require__(
            '../../node_modules/.pnpm/@mui+utils@5.8.6_react@18.2.0/node_modules/@mui/utils/esm/composeClasses/composeClasses.js',
          ),
          useThemeProps = __webpack_require__(
            '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/styles/useThemeProps.js',
          ),
          styled = __webpack_require__(
            '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/styles/styled.js',
          ),
          generateUtilityClass = __webpack_require__(
            '../../node_modules/.pnpm/@mui+utils@5.8.6_react@18.2.0/node_modules/@mui/utils/esm/generateUtilityClass/generateUtilityClass.js',
          );
        function getTableContainerUtilityClass(slot) {
          return (0, generateUtilityClass.A)('MuiTableContainer', slot);
        }
        (0,
        __webpack_require__(
          '../../node_modules/.pnpm/@mui+utils@5.8.6_react@18.2.0/node_modules/@mui/utils/esm/generateUtilityClasses/generateUtilityClasses.js',
        ).A)('MuiTableContainer', ['root']);
        var jsx_runtime = __webpack_require__(
          '../../node_modules/.pnpm/react@18.2.0/node_modules/react/jsx-runtime.js',
        );
        const _excluded = ['className', 'component'],
          TableContainerRoot = (0, styled.Ay)('div', {
            name: 'MuiTableContainer',
            slot: 'Root',
            overridesResolver: (props, styles) => styles.root,
          })({ width: '100%', overflowX: 'auto' }),
          TableContainer_TableContainer = react.forwardRef(function TableContainer(inProps, ref) {
            const props = (0, useThemeProps.A)({ props: inProps, name: 'MuiTableContainer' }),
              { className, component = 'div' } = props,
              other = (0, objectWithoutPropertiesLoose.A)(props, _excluded),
              ownerState = (0, esm_extends.A)({}, props, { component }),
              classes = ((ownerState) => {
                const { classes } = ownerState;
                return (0, composeClasses.A)(
                  { root: ['root'] },
                  getTableContainerUtilityClass,
                  classes,
                );
              })(ownerState);
            return (0, jsx_runtime.jsx)(
              TableContainerRoot,
              (0, esm_extends.A)(
                {
                  ref,
                  as: component,
                  className: (0, clsx_m.default)(classes.root, className),
                  ownerState,
                },
                other,
              ),
            );
          });
      },
    './src/components/admin/permissions-card.stories.tsx': (
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
      __webpack_require__('../../node_modules/.pnpm/react@18.2.0/node_modules/react/index.js');
      var _components_admin_permissions_card__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(
          './src/components/admin/permissions-card.tsx',
        ),
        react_jsx_runtime__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(
          '../../node_modules/.pnpm/react@18.2.0/node_modules/react/jsx-runtime.js',
        );
      const __WEBPACK_DEFAULT_EXPORT__ = {
          title: 'Admin/Permissions Card',
          component: _components_admin_permissions_card__WEBPACK_IMPORTED_MODULE_1__.f,
          argTypes: {
            permissions: {
              defaultValue: [
                { action: 'task_submit', authz_grp: 'group1' },
                { action: 'task_read', authz_grp: 'group1' },
              ],
            },
          },
        },
        Default = (args) =>
          (0, react_jsx_runtime__WEBPACK_IMPORTED_MODULE_2__.jsx)(
            _components_admin_permissions_card__WEBPACK_IMPORTED_MODULE_1__.f,
            { ...args, savePermission: () => new Promise((res) => setTimeout(res, 100)) },
          );
      (Default.storyName = 'Permissions Card'),
        (Default.parameters = {
          ...Default.parameters,
          docs: {
            ...Default.parameters?.docs,
            source: {
              originalSource:
                'args => {\n  return <PermissionsCard {...args} savePermission={() => new Promise(res => setTimeout(res, 100))} />;\n}',
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
