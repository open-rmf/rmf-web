'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [71999],
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
    '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/Search.js':
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
              d: 'M15.5 14h-.79l-.28-.27C15.41 12.59 16 11.11 16 9.5 16 5.91 13.09 3 9.5 3S3 5.91 3 9.5 5.91 16 9.5 16c1.61 0 3.09-.59 4.23-1.57l.27.28v.79l5 4.99L20.49 19l-4.99-5zm-6 0C7.01 14 5 11.99 5 9.5S7.01 5 9.5 5 14 7.01 14 9.5 11.99 14 9.5 14z',
            }),
            'Search',
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
    './src/components/admin/user-list-card.stories.tsx': (
      __unused_webpack_module,
      __webpack_exports__,
      __webpack_require__,
    ) => {
      __webpack_require__.r(__webpack_exports__),
        __webpack_require__.d(__webpack_exports__, {
          Default: () => Default,
          __namedExportsOrder: () => __namedExportsOrder,
          default: () => user_list_card_stories,
        });
      var dist = __webpack_require__(
          '../../node_modules/.pnpm/react-router@6.14.1_react@18.2.0/node_modules/react-router/dist/index.js',
        ),
        styled = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/styles/styled.js',
        ),
        Card = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Card/Card.js',
        ),
        CardHeader = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/CardHeader/CardHeader.js',
        ),
        TextField = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/TextField/TextField.js',
        ),
        InputAdornment = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/InputAdornment/InputAdornment.js',
        ),
        IconButton = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/IconButton/IconButton.js',
        ),
        TableContainer = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/TableContainer/TableContainer.js',
        ),
        Table = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Table/Table.js',
        ),
        TableHead = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/TableHead/TableHead.js',
        ),
        TableRow = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/TableRow/TableRow.js',
        ),
        TableCell = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/TableCell/TableCell.js',
        ),
        TableBody = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/TableBody/TableBody.js',
        ),
        Button = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Button/Button.js',
        ),
        TablePagination = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/TablePagination/TablePagination.js',
        ),
        Typography = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Typography/Typography.js',
        ),
        AccountCircle = __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/AccountCircle.js',
        ),
        AddCircle = __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/AddCircle.js',
        ),
        Delete = __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/Delete.js',
        ),
        Search = __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/Search.js',
        ),
        react = __webpack_require__(
          '../../node_modules/.pnpm/react@18.2.0/node_modules/react/index.js',
        ),
        react_components_dist = __webpack_require__('../react-components/dist/index.js'),
        app_contexts = __webpack_require__('./src/components/app-contexts.tsx'),
        create_user_dialog = __webpack_require__('./src/components/admin/create-user-dialog.tsx'),
        jsx_runtime = __webpack_require__(
          '../../node_modules/.pnpm/react@18.2.0/node_modules/react/jsx-runtime.js',
        );
      const classes = {
          controlsButton: ''.concat('user-list-card', '-controls-button'),
          tableRow: ''.concat('user-list-card', '-table-row'),
        },
        StyledCard = (0, styled.Ay)((props) => (0, jsx_runtime.jsx)(Card.A, { ...props }))(
          (_ref) => {
            let { theme } = _ref;
            return {
              ['& .'.concat(classes.controlsButton)]: { float: 'right' },
              ['& .'.concat(classes.tableRow)]: {
                cursor: 'pointer',
                '&:hover': { backgroundColor: theme.palette.action.hover },
              },
            };
          },
        );
      function UserListCard(_ref2) {
        let { searchUsers, deleteUser, createUser } = _ref2;
        const safeAsync = (0, react_components_dist.Yb)(),
          navigate = (0, dist.Zp)(),
          [users, setUsers] = react.useState([]),
          [selectedUser, setSelectedUser] = react.useState(null),
          [search, setSearch] = react.useState(''),
          [searching, setSearching] = react.useState(!1),
          [searchInput, setSearchInput] = react.useState(''),
          searchTimer = react.useRef(void 0),
          [page, setPage] = react.useState(0),
          [hasMore, setHasMore] = react.useState(!1),
          [openDeleteDialog, setOpenDeleteDialog] = react.useState(!1),
          [deleting, setDeleting] = react.useState(!1),
          [openCreateDialog, setOpenCreateDialog] = react.useState(!1),
          { showAlert } = react.useContext(app_contexts.J3),
          refresh = react.useCallback(async () => {
            searchUsers &&
              (setSearching(!0),
              (async () => {
                try {
                  const results = await safeAsync(searchUsers(search, 21, 20 * page));
                  setHasMore(results.length > 20), setUsers(results.slice(0, 20));
                } catch (e) {
                  showAlert('error', 'Failed to get users: '.concat(e.message));
                } finally {
                  setSearching(!1);
                }
              })());
          }, [page, search, searchUsers, showAlert, safeAsync]);
        return (
          react.useEffect(() => {
            refresh();
          }, [refresh]),
          (0, jsx_runtime.jsxs)(StyledCard, {
            variant: 'outlined',
            children: [
              (0, jsx_runtime.jsx)(CardHeader.A, {
                title: 'Users',
                titleTypographyProps: { variant: 'h5' },
                avatar: (0, jsx_runtime.jsx)(AccountCircle.A, { fontSize: 'large' }),
                action: (0, jsx_runtime.jsxs)(jsx_runtime.Fragment, {
                  children: [
                    (0, jsx_runtime.jsx)(TextField.A, {
                      variant: 'outlined',
                      id: 'search-users',
                      label: 'Search Users',
                      InputProps: {
                        startAdornment: (0, jsx_runtime.jsx)(InputAdornment.A, {
                          position: 'start',
                          children: (0, jsx_runtime.jsx)(Search.A, {}),
                        }),
                      },
                      value: searchInput,
                      onChange: (ev) => {
                        const newInput = ev.target.value;
                        setSearchInput(newInput),
                          clearTimeout(searchTimer.current),
                          (searchTimer.current = window.setTimeout(() => setSearch(newInput), 300));
                      },
                    }),
                    (0, jsx_runtime.jsx)(IconButton.A, {
                      onClick: () => setOpenCreateDialog(!0),
                      'aria-label': 'create user',
                      children: (0, jsx_runtime.jsx)(AddCircle.A, { fontSize: 'large' }),
                    }),
                  ],
                }),
              }),
              (0, jsx_runtime.jsxs)(TableContainer.A, {
                id: 'admin-user-table',
                children: [
                  (0, jsx_runtime.jsxs)(react_components_dist.Rh, {
                    loading: searching,
                    children: [
                      (0, jsx_runtime.jsxs)(Table.A, {
                        size: 'small',
                        children: [
                          (0, jsx_runtime.jsx)(TableHead.A, {
                            children: (0, jsx_runtime.jsxs)(TableRow.A, {
                              children: [
                                (0, jsx_runtime.jsx)(TableCell.A, { children: 'Username' }),
                                (0, jsx_runtime.jsx)(TableCell.A, {}),
                              ],
                            }),
                          }),
                          (0, jsx_runtime.jsx)(TableBody.A, {
                            children: users.map((u) =>
                              (0, jsx_runtime.jsxs)(
                                TableRow.A,
                                {
                                  className: classes.tableRow,
                                  onClick: () => navigate(''.concat(u)),
                                  children: [
                                    (0, jsx_runtime.jsx)(TableCell.A, { children: u }),
                                    (0, jsx_runtime.jsx)(TableCell.A, {
                                      children: (0, jsx_runtime.jsx)(Button.A, {
                                        variant: 'contained',
                                        color: 'secondary',
                                        startIcon: (0, jsx_runtime.jsx)(Delete.A, {}),
                                        className: classes.controlsButton,
                                        onClick: (ev) => {
                                          ev.stopPropagation(),
                                            setSelectedUser(u),
                                            setOpenDeleteDialog(!0);
                                        },
                                        children: 'Delete',
                                      }),
                                    }),
                                  ],
                                },
                                u,
                              ),
                            ),
                          }),
                        ],
                      }),
                      0 === users.length &&
                        searching &&
                        (0, jsx_runtime.jsx)('div', { style: { height: 100 } }),
                    ],
                  }),
                  (0, jsx_runtime.jsx)(TablePagination.A, {
                    component: 'div',
                    count: hasMore ? -1 : 20 * page + users.length,
                    page,
                    rowsPerPage: 20,
                    rowsPerPageOptions: [20],
                    onPageChange: (_, newPage) => setPage(newPage),
                  }),
                ],
              }),
              openDeleteDialog &&
                (0, jsx_runtime.jsx)(react_components_dist.KK, {
                  open: openDeleteDialog,
                  title: 'Confirm Delete',
                  submitting: deleting,
                  onClose: () => setOpenDeleteDialog(!1),
                  onSubmit: async () => {
                    setDeleting(!0);
                    try {
                      selectedUser && deleteUser && (await safeAsync(deleteUser(selectedUser))),
                        setDeleting(!1),
                        setOpenDeleteDialog(!1),
                        refresh();
                    } catch (e) {
                      setDeleting(!1),
                        showAlert('error', 'Failed to delete user: '.concat(e.message));
                    }
                  },
                  children: (0, jsx_runtime.jsx)(Typography.A, {
                    children: 'Are you sure you want to delete "'.concat(selectedUser, '"?'),
                  }),
                }),
              openCreateDialog &&
                (0, jsx_runtime.jsx)(create_user_dialog._, {
                  open: openCreateDialog,
                  setOpen: setOpenCreateDialog,
                  createUser:
                    createUser &&
                    (async (user) => {
                      await createUser(user), refresh();
                    }),
                }),
            ],
          })
        );
      }
      UserListCard.__docgenInfo = {
        description: '',
        methods: [],
        displayName: 'UserListCard',
        props: {
          searchUsers: {
            required: !1,
            tsType: {
              name: 'signature',
              type: 'function',
              raw: '(search: string, limit: number, offset: number) => Promise<string[]> | string[]',
              signature: {
                arguments: [
                  { type: { name: 'string' }, name: 'search' },
                  { type: { name: 'number' }, name: 'limit' },
                  { type: { name: 'number' }, name: 'offset' },
                ],
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
          deleteUser: {
            required: !1,
            tsType: {
              name: 'signature',
              type: 'function',
              raw: '(user: string) => Promise<void> | void',
              signature: {
                arguments: [{ type: { name: 'string' }, name: 'user' }],
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
      const user_list_card_stories = { title: 'Admin/User List Card', component: UserListCard },
        users = [];
      for (let i = 0; i < 100; i++) users.push('user'.concat(i + 1));
      async function searchUsers(search, limit, offset) {
        return (
          await new Promise((res) => setTimeout(res, 100)),
          users.filter((u) => u.startsWith(search)).slice(offset, offset + limit)
        );
      }
      const Default = (args) =>
        (0, jsx_runtime.jsx)(dist.fS, {
          children: (0, jsx_runtime.jsx)(UserListCard, {
            ...args,
            searchUsers,
            deleteUser: () => new Promise((res) => setTimeout(res, 100)),
            createUser: () => new Promise((res) => setTimeout(res, 100)),
          }),
        });
      (Default.storyName = 'User List Card'),
        (Default.parameters = {
          ...Default.parameters,
          docs: {
            ...Default.parameters?.docs,
            source: {
              originalSource:
                'args => {\n  return <MemoryRouter>\n      <UserListCard {...args} searchUsers={searchUsers} deleteUser={() => new Promise(res => setTimeout(res, 100))} createUser={() => new Promise(res => setTimeout(res, 100))} />\n    </MemoryRouter>;\n}',
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
