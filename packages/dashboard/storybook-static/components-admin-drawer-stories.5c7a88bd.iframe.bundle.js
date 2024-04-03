'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [50741],
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
    './src/components/admin/drawer.stories.tsx': (
      __unused_webpack_module,
      __webpack_exports__,
      __webpack_require__,
    ) => {
      __webpack_require__.r(__webpack_exports__),
        __webpack_require__.d(__webpack_exports__, {
          Default: () => Default,
          __namedExportsOrder: () => __namedExportsOrder,
          default: () => drawer_stories,
        });
      var styled = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/styles/styled.js',
        ),
        objectWithoutPropertiesLoose = __webpack_require__(
          '../../node_modules/.pnpm/@babel+runtime@7.18.6/node_modules/@babel/runtime/helpers/esm/objectWithoutPropertiesLoose.js',
        ),
        esm_extends = __webpack_require__(
          '../../node_modules/.pnpm/@babel+runtime@7.18.6/node_modules/@babel/runtime/helpers/esm/extends.js',
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
        Modal = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Modal/Modal.js',
        ),
        Transition = __webpack_require__(
          '../../node_modules/.pnpm/react-transition-group@4.4.2_react-dom@18.2.0_react@18.2.0/node_modules/react-transition-group/esm/Transition.js',
        ),
        debounce = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/utils/debounce.js',
        ),
        useForkRef = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/utils/useForkRef.js',
        ),
        useTheme = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/styles/useTheme.js',
        ),
        utils = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/transitions/utils.js',
        ),
        ownerWindow = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/utils/ownerWindow.js',
        ),
        jsx_runtime = __webpack_require__(
          '../../node_modules/.pnpm/react@18.2.0/node_modules/react/jsx-runtime.js',
        );
      const _excluded = [
        'addEndListener',
        'appear',
        'children',
        'container',
        'direction',
        'easing',
        'in',
        'onEnter',
        'onEntered',
        'onEntering',
        'onExit',
        'onExited',
        'onExiting',
        'style',
        'timeout',
        'TransitionComponent',
      ];
      function setTranslateValue(direction, node, containerProp) {
        const transform = (function getTranslateValue(direction, node, resolvedContainer) {
          const rect = node.getBoundingClientRect(),
            containerRect = resolvedContainer && resolvedContainer.getBoundingClientRect(),
            containerWindow = (0, ownerWindow.A)(node);
          let transform;
          if (node.fakeTransform) transform = node.fakeTransform;
          else {
            const computedStyle = containerWindow.getComputedStyle(node);
            transform =
              computedStyle.getPropertyValue('-webkit-transform') ||
              computedStyle.getPropertyValue('transform');
          }
          let offsetX = 0,
            offsetY = 0;
          if (transform && 'none' !== transform && 'string' == typeof transform) {
            const transformValues = transform.split('(')[1].split(')')[0].split(',');
            (offsetX = parseInt(transformValues[4], 10)),
              (offsetY = parseInt(transformValues[5], 10));
          }
          return 'left' === direction
            ? containerRect
              ? `translateX(${containerRect.right + offsetX - rect.left}px)`
              : `translateX(${containerWindow.innerWidth + offsetX - rect.left}px)`
            : 'right' === direction
              ? containerRect
                ? `translateX(-${rect.right - containerRect.left - offsetX}px)`
                : `translateX(-${rect.left + rect.width - offsetX}px)`
              : 'up' === direction
                ? containerRect
                  ? `translateY(${containerRect.bottom + offsetY - rect.top}px)`
                  : `translateY(${containerWindow.innerHeight + offsetY - rect.top}px)`
                : containerRect
                  ? `translateY(-${rect.top - containerRect.top + rect.height - offsetY}px)`
                  : `translateY(-${rect.top + rect.height - offsetY}px)`;
        })(
          direction,
          node,
          (function resolveContainer(containerPropProp) {
            return 'function' == typeof containerPropProp ? containerPropProp() : containerPropProp;
          })(containerProp),
        );
        transform && ((node.style.webkitTransform = transform), (node.style.transform = transform));
      }
      const Slide_Slide = react.forwardRef(function Slide(props, ref) {
        const theme = (0, useTheme.A)(),
          defaultEasing = {
            enter: theme.transitions.easing.easeOut,
            exit: theme.transitions.easing.sharp,
          },
          defaultTimeout = {
            enter: theme.transitions.duration.enteringScreen,
            exit: theme.transitions.duration.leavingScreen,
          },
          {
            addEndListener,
            appear = !0,
            children,
            container: containerProp,
            direction = 'down',
            easing: easingProp = defaultEasing,
            in: inProp,
            onEnter,
            onEntered,
            onEntering,
            onExit,
            onExited,
            onExiting,
            style,
            timeout = defaultTimeout,
            TransitionComponent = Transition.Ay,
          } = props,
          other = (0, objectWithoutPropertiesLoose.A)(props, _excluded),
          childrenRef = react.useRef(null),
          handleRefIntermediary = (0, useForkRef.A)(children.ref, childrenRef),
          handleRef = (0, useForkRef.A)(handleRefIntermediary, ref),
          normalizedTransitionCallback = (callback) => (isAppearing) => {
            callback &&
              (void 0 === isAppearing
                ? callback(childrenRef.current)
                : callback(childrenRef.current, isAppearing));
          },
          handleEnter = normalizedTransitionCallback((node, isAppearing) => {
            setTranslateValue(direction, node, containerProp),
              (0, utils.q)(node),
              onEnter && onEnter(node, isAppearing);
          }),
          handleEntering = normalizedTransitionCallback((node, isAppearing) => {
            const transitionProps = (0, utils.c)(
              { timeout, style, easing: easingProp },
              { mode: 'enter' },
            );
            (node.style.webkitTransition = theme.transitions.create(
              '-webkit-transform',
              (0, esm_extends.A)({}, transitionProps),
            )),
              (node.style.transition = theme.transitions.create(
                'transform',
                (0, esm_extends.A)({}, transitionProps),
              )),
              (node.style.webkitTransform = 'none'),
              (node.style.transform = 'none'),
              onEntering && onEntering(node, isAppearing);
          }),
          handleEntered = normalizedTransitionCallback(onEntered),
          handleExiting = normalizedTransitionCallback(onExiting),
          handleExit = normalizedTransitionCallback((node) => {
            const transitionProps = (0, utils.c)(
              { timeout, style, easing: easingProp },
              { mode: 'exit' },
            );
            (node.style.webkitTransition = theme.transitions.create(
              '-webkit-transform',
              transitionProps,
            )),
              (node.style.transition = theme.transitions.create('transform', transitionProps)),
              setTranslateValue(direction, node, containerProp),
              onExit && onExit(node);
          }),
          handleExited = normalizedTransitionCallback((node) => {
            (node.style.webkitTransition = ''),
              (node.style.transition = ''),
              onExited && onExited(node);
          }),
          updatePosition = react.useCallback(() => {
            childrenRef.current && setTranslateValue(direction, childrenRef.current, containerProp);
          }, [direction, containerProp]);
        return (
          react.useEffect(() => {
            if (inProp || 'down' === direction || 'right' === direction) return;
            const handleResize = (0, debounce.A)(() => {
                childrenRef.current &&
                  setTranslateValue(direction, childrenRef.current, containerProp);
              }),
              containerWindow = (0, ownerWindow.A)(childrenRef.current);
            return (
              containerWindow.addEventListener('resize', handleResize),
              () => {
                handleResize.clear(), containerWindow.removeEventListener('resize', handleResize);
              }
            );
          }, [direction, inProp, containerProp]),
          react.useEffect(() => {
            inProp || updatePosition();
          }, [inProp, updatePosition]),
          (0, jsx_runtime.jsx)(
            TransitionComponent,
            (0, esm_extends.A)(
              {
                nodeRef: childrenRef,
                onEnter: handleEnter,
                onEntered: handleEntered,
                onEntering: handleEntering,
                onExit: handleExit,
                onExited: handleExited,
                onExiting: handleExiting,
                addEndListener: (next) => {
                  addEndListener && addEndListener(childrenRef.current, next);
                },
                appear,
                in: inProp,
                timeout,
              },
              other,
              {
                children: (state, childProps) =>
                  react.cloneElement(
                    children,
                    (0, esm_extends.A)(
                      {
                        ref: handleRef,
                        style: (0, esm_extends.A)(
                          { visibility: 'exited' !== state || inProp ? void 0 : 'hidden' },
                          style,
                          children.props.style,
                        ),
                      },
                      childProps,
                    ),
                  ),
              },
            ),
          )
        );
      });
      var Paper = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Paper/Paper.js',
        ),
        capitalize = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/utils/capitalize.js',
        ),
        useThemeProps = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/styles/useThemeProps.js',
        ),
        generateUtilityClass = __webpack_require__(
          '../../node_modules/.pnpm/@mui+utils@5.8.6_react@18.2.0/node_modules/@mui/utils/esm/generateUtilityClass/generateUtilityClass.js',
        );
      function getDrawerUtilityClass(slot) {
        return (0, generateUtilityClass.A)('MuiDrawer', slot);
      }
      (0,
      __webpack_require__(
        '../../node_modules/.pnpm/@mui+utils@5.8.6_react@18.2.0/node_modules/@mui/utils/esm/generateUtilityClasses/generateUtilityClasses.js',
      ).A)('MuiDrawer', [
        'root',
        'docked',
        'paper',
        'paperAnchorLeft',
        'paperAnchorRight',
        'paperAnchorTop',
        'paperAnchorBottom',
        'paperAnchorDockedLeft',
        'paperAnchorDockedRight',
        'paperAnchorDockedTop',
        'paperAnchorDockedBottom',
        'modal',
      ]);
      const Drawer_excluded = ['BackdropProps'],
        _excluded2 = [
          'anchor',
          'BackdropProps',
          'children',
          'className',
          'elevation',
          'hideBackdrop',
          'ModalProps',
          'onClose',
          'open',
          'PaperProps',
          'SlideProps',
          'TransitionComponent',
          'transitionDuration',
          'variant',
        ],
        overridesResolver = (props, styles) => {
          const { ownerState } = props;
          return [
            styles.root,
            ('permanent' === ownerState.variant || 'persistent' === ownerState.variant) &&
              styles.docked,
            styles.modal,
          ];
        },
        DrawerRoot = (0, styled.Ay)(Modal.A, {
          name: 'MuiDrawer',
          slot: 'Root',
          overridesResolver,
        })(({ theme }) => ({ zIndex: (theme.vars || theme).zIndex.drawer })),
        DrawerDockedRoot = (0, styled.Ay)('div', {
          shouldForwardProp: styled.ep,
          name: 'MuiDrawer',
          slot: 'Docked',
          skipVariantsResolver: !1,
          overridesResolver,
        })({ flex: '0 0 auto' }),
        DrawerPaper = (0, styled.Ay)(Paper.A, {
          name: 'MuiDrawer',
          slot: 'Paper',
          overridesResolver: (props, styles) => {
            const { ownerState } = props;
            return [
              styles.paper,
              styles[`paperAnchor${(0, capitalize.A)(ownerState.anchor)}`],
              'temporary' !== ownerState.variant &&
                styles[`paperAnchorDocked${(0, capitalize.A)(ownerState.anchor)}`],
            ];
          },
        })(({ theme, ownerState }) =>
          (0, esm_extends.A)(
            {
              overflowY: 'auto',
              display: 'flex',
              flexDirection: 'column',
              height: '100%',
              flex: '1 0 auto',
              zIndex: (theme.vars || theme).zIndex.drawer,
              WebkitOverflowScrolling: 'touch',
              position: 'fixed',
              top: 0,
              outline: 0,
            },
            'left' === ownerState.anchor && { left: 0 },
            'top' === ownerState.anchor && {
              top: 0,
              left: 0,
              right: 0,
              height: 'auto',
              maxHeight: '100%',
            },
            'right' === ownerState.anchor && { right: 0 },
            'bottom' === ownerState.anchor && {
              top: 'auto',
              left: 0,
              bottom: 0,
              right: 0,
              height: 'auto',
              maxHeight: '100%',
            },
            'left' === ownerState.anchor &&
              'temporary' !== ownerState.variant && {
                borderRight: `1px solid ${(theme.vars || theme).palette.divider}`,
              },
            'top' === ownerState.anchor &&
              'temporary' !== ownerState.variant && {
                borderBottom: `1px solid ${(theme.vars || theme).palette.divider}`,
              },
            'right' === ownerState.anchor &&
              'temporary' !== ownerState.variant && {
                borderLeft: `1px solid ${(theme.vars || theme).palette.divider}`,
              },
            'bottom' === ownerState.anchor &&
              'temporary' !== ownerState.variant && {
                borderTop: `1px solid ${(theme.vars || theme).palette.divider}`,
              },
          ),
        ),
        oppositeDirection = { left: 'right', right: 'left', top: 'down', bottom: 'up' };
      const Drawer = react.forwardRef(function Drawer(inProps, ref) {
          const props = (0, useThemeProps.A)({ props: inProps, name: 'MuiDrawer' }),
            theme = (0, useTheme.A)(),
            defaultTransitionDuration = {
              enter: theme.transitions.duration.enteringScreen,
              exit: theme.transitions.duration.leavingScreen,
            },
            {
              anchor: anchorProp = 'left',
              BackdropProps,
              children,
              className,
              elevation = 16,
              hideBackdrop = !1,
              ModalProps: { BackdropProps: BackdropPropsProp } = {},
              onClose,
              open = !1,
              PaperProps = {},
              SlideProps,
              TransitionComponent = Slide_Slide,
              transitionDuration = defaultTransitionDuration,
              variant = 'temporary',
            } = props,
            ModalProps = (0, objectWithoutPropertiesLoose.A)(props.ModalProps, Drawer_excluded),
            other = (0, objectWithoutPropertiesLoose.A)(props, _excluded2),
            mounted = react.useRef(!1);
          react.useEffect(() => {
            mounted.current = !0;
          }, []);
          const anchorInvariant = (function getAnchor(theme, anchor) {
              return 'rtl' === theme.direction &&
                (function isHorizontal(anchor) {
                  return -1 !== ['left', 'right'].indexOf(anchor);
                })(anchor)
                ? oppositeDirection[anchor]
                : anchor;
            })(theme, anchorProp),
            anchor = anchorProp,
            ownerState = (0, esm_extends.A)({}, props, { anchor, elevation, open, variant }, other),
            classes = ((ownerState) => {
              const { classes, anchor, variant } = ownerState,
                slots = {
                  root: ['root'],
                  docked: [('permanent' === variant || 'persistent' === variant) && 'docked'],
                  modal: ['modal'],
                  paper: [
                    'paper',
                    `paperAnchor${(0, capitalize.A)(anchor)}`,
                    'temporary' !== variant && `paperAnchorDocked${(0, capitalize.A)(anchor)}`,
                  ],
                };
              return (0, composeClasses.A)(slots, getDrawerUtilityClass, classes);
            })(ownerState),
            drawer = (0, jsx_runtime.jsx)(
              DrawerPaper,
              (0, esm_extends.A)(
                { elevation: 'temporary' === variant ? elevation : 0, square: !0 },
                PaperProps,
                {
                  className: (0, clsx_m.default)(classes.paper, PaperProps.className),
                  ownerState,
                  children,
                },
              ),
            );
          if ('permanent' === variant)
            return (0, jsx_runtime.jsx)(
              DrawerDockedRoot,
              (0, esm_extends.A)(
                {
                  className: (0, clsx_m.default)(classes.root, classes.docked, className),
                  ownerState,
                  ref,
                },
                other,
                { children: drawer },
              ),
            );
          const slidingDrawer = (0, jsx_runtime.jsx)(
            TransitionComponent,
            (0, esm_extends.A)(
              {
                in: open,
                direction: oppositeDirection[anchorInvariant],
                timeout: transitionDuration,
                appear: mounted.current,
              },
              SlideProps,
              { children: drawer },
            ),
          );
          return 'persistent' === variant
            ? (0, jsx_runtime.jsx)(
                DrawerDockedRoot,
                (0, esm_extends.A)(
                  {
                    className: (0, clsx_m.default)(classes.root, classes.docked, className),
                    ownerState,
                    ref,
                  },
                  other,
                  { children: slidingDrawer },
                ),
              )
            : (0, jsx_runtime.jsx)(
                DrawerRoot,
                (0, esm_extends.A)(
                  {
                    BackdropProps: (0, esm_extends.A)({}, BackdropProps, BackdropPropsProp, {
                      transitionDuration,
                    }),
                    className: (0, clsx_m.default)(classes.root, classes.modal, className),
                    open,
                    ownerState,
                    onClose,
                    hideBackdrop,
                    ref,
                  },
                  other,
                  ModalProps,
                  { children: slidingDrawer },
                ),
              );
        }),
        Drawer_Drawer = Drawer;
      var ListItem = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/ListItem/ListItem.js',
        ),
        ListItemIcon = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/ListItemIcon/ListItemIcon.js',
        ),
        ListItemText = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/ListItemText/ListItemText.js',
        ),
        Toolbar = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Toolbar/Toolbar.js',
        ),
        List = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/List/List.js',
        ),
        AccountCircle = __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/AccountCircle.js',
        ),
        Security = __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/Security.js',
        ),
        dist = __webpack_require__(
          '../../node_modules/.pnpm/react-router@6.14.1_react@18.2.0/node_modules/react-router/dist/index.js',
        );
      const drawerValuesRoutesMap = { Users: { path: '/users' }, Roles: { path: '/roles' } },
        classes = {
          drawerPaper: ''.concat('drawer', '-paper'),
          drawerContainer: ''.concat('drawer', '-container'),
          itemIcon: ''.concat('drawer', '-itemicon'),
          activeItem: ''.concat('drawer', '-active-item'),
        },
        StyledDrawer = (0, styled.Ay)((props) => (0, jsx_runtime.jsx)(Drawer_Drawer, { ...props }))(
          (_ref) => {
            let { theme } = _ref;
            return {
              ['& .'.concat(classes.drawerPaper)]: {
                backgroundColor: theme.palette.primary.dark,
                color: theme.palette.getContrastText(theme.palette.primary.dark),
                minWidth: 240,
                width: '16%',
              },
              ['& .'.concat(classes.drawerContainer)]: { overflow: 'auto' },
              ['& .'.concat(classes.itemIcon)]: {
                color: theme.palette.getContrastText(theme.palette.primary.dark),
              },
              ['& .'.concat(classes.activeItem)]: {
                backgroundColor: ''.concat(theme.palette.primary.light, ' !important'),
              },
            };
          },
        );
      function AdminDrawer() {
        const location = (0, dist.zy)(),
          navigate = (0, dist.Zp)(),
          activeItem = react.useMemo(() => {
            const matched = Object.entries(drawerValuesRoutesMap).find((_ref2) => {
              let [, v] = _ref2;
              return location.pathname === '/admin'.concat(v.path);
            });
            return matched ? matched[0] : 'Users';
          }, [location.pathname]),
          DrawerItem = react.useCallback(
            (_ref3) => {
              let { Icon, text, route } = _ref3;
              return (0, jsx_runtime.jsxs)(ListItem.Ay, {
                button: !0,
                className: activeItem === text ? classes.activeItem : void 0,
                onClick: () => {
                  navigate(route);
                },
                children: [
                  (0, jsx_runtime.jsx)(ListItemIcon.A, {
                    children: (0, jsx_runtime.jsx)(Icon, { className: classes.itemIcon }),
                  }),
                  (0, jsx_runtime.jsx)(ListItemText.A, { children: text }),
                ],
              });
            },
            [activeItem, navigate],
          );
        return (0, jsx_runtime.jsxs)(StyledDrawer, {
          variant: 'permanent',
          classes: { paper: classes.drawerPaper },
          children: [
            (0, jsx_runtime.jsx)(Toolbar.A, {}),
            (0, jsx_runtime.jsx)('div', {
              className: classes.drawerContainer,
              children: (0, jsx_runtime.jsxs)(List.A, {
                children: [
                  (0, jsx_runtime.jsx)(DrawerItem, {
                    text: 'Users',
                    route: 'users',
                    Icon: AccountCircle.A,
                  }),
                  (0, jsx_runtime.jsx)(DrawerItem, {
                    text: 'Roles',
                    route: 'roles',
                    Icon: Security.A,
                  }),
                ],
              }),
            }),
          ],
        });
      }
      AdminDrawer.__docgenInfo = { description: '', methods: [], displayName: 'AdminDrawer' };
      const drawer_stories = {
          title: 'Admin/Drawer',
          component: AdminDrawer,
          argTypes: { active: { defaultValue: 'Users' } },
        },
        Default = (args) => (0, jsx_runtime.jsx)(AdminDrawer, { ...args });
      (Default.storyName = 'Drawer'),
        (Default.parameters = {
          ...Default.parameters,
          docs: {
            ...Default.parameters?.docs,
            source: {
              originalSource: 'args => {\n  return <AdminDrawer {...args} />;\n}',
              ...Default.parameters?.docs?.source,
            },
          },
        });
      const __namedExportsOrder = ['Default'];
    },
  },
]);
