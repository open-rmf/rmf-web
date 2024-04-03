/*! For license information please see 96213.8e70eabd.iframe.bundle.js.LICENSE.txt */
'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [96213],
  {
    '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/AccountCircle.js':
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
              d: 'M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm0 3c1.66 0 3 1.34 3 3s-1.34 3-3 3-3-1.34-3-3 1.34-3 3-3zm0 14.2c-2.5 0-4.71-1.28-6-3.22.03-1.99 4-3.08 6-3.08 1.99 0 5.97 1.09 6 3.08-1.29 1.94-3.5 3.22-6 3.22z',
            }),
            'AccountCircle',
          );
        exports.A = _default;
      },
    '../../node_modules/.pnpm/react-router@6.14.1_react@18.2.0/node_modules/react-router/dist/index.js':
      (__unused_webpack_module, __webpack_exports__, __webpack_require__) => {
        __webpack_require__.d(__webpack_exports__, {
          fS: () => MemoryRouter,
          zy: () => useLocation,
          Zp: () => useNavigate,
        });
        var router_Action,
          react = __webpack_require__(
            '../../node_modules/.pnpm/react@18.2.0/node_modules/react/index.js',
          ),
          react_namespaceObject = __webpack_require__.t(react, 2);
        function _extends() {
          return (
            (_extends = Object.assign
              ? Object.assign.bind()
              : function (target) {
                  for (var i = 1; i < arguments.length; i++) {
                    var source = arguments[i];
                    for (var key in source)
                      Object.prototype.hasOwnProperty.call(source, key) &&
                        (target[key] = source[key]);
                  }
                  return target;
                }),
            _extends.apply(this, arguments)
          );
        }
        !(function (Action) {
          (Action.Pop = 'POP'), (Action.Push = 'PUSH'), (Action.Replace = 'REPLACE');
        })(router_Action || (router_Action = {}));
        function invariant(value, message) {
          if (!1 === value || null == value) throw new Error(message);
        }
        function warning(cond, message) {
          if (!cond) {
            'undefined' != typeof console && console.warn(message);
            try {
              throw new Error(message);
            } catch (e) {}
          }
        }
        function createLocation(current, to, state, key) {
          return (
            void 0 === state && (state = null),
            _extends(
              {
                pathname: 'string' == typeof current ? current : current.pathname,
                search: '',
                hash: '',
              },
              'string' == typeof to ? router_parsePath(to) : to,
              { state, key: (to && to.key) || key || Math.random().toString(36).substr(2, 8) },
            )
          );
        }
        function createPath(_ref) {
          let { pathname = '/', search = '', hash = '' } = _ref;
          return (
            search &&
              '?' !== search &&
              (pathname += '?' === search.charAt(0) ? search : '?' + search),
            hash && '#' !== hash && (pathname += '#' === hash.charAt(0) ? hash : '#' + hash),
            pathname
          );
        }
        function router_parsePath(path) {
          let parsedPath = {};
          if (path) {
            let hashIndex = path.indexOf('#');
            hashIndex >= 0 &&
              ((parsedPath.hash = path.substr(hashIndex)), (path = path.substr(0, hashIndex)));
            let searchIndex = path.indexOf('?');
            searchIndex >= 0 &&
              ((parsedPath.search = path.substr(searchIndex)),
              (path = path.substr(0, searchIndex))),
              path && (parsedPath.pathname = path);
          }
          return parsedPath;
        }
        var ResultType;
        !(function (ResultType) {
          (ResultType.data = 'data'),
            (ResultType.deferred = 'deferred'),
            (ResultType.redirect = 'redirect'),
            (ResultType.error = 'error');
        })(ResultType || (ResultType = {}));
        new Set(['lazy', 'caseSensitive', 'path', 'id', 'index', 'children']);
        function router_stripBasename(pathname, basename) {
          if ('/' === basename) return pathname;
          if (!pathname.toLowerCase().startsWith(basename.toLowerCase())) return null;
          let startIndex = basename.endsWith('/') ? basename.length - 1 : basename.length,
            nextChar = pathname.charAt(startIndex);
          return nextChar && '/' !== nextChar ? null : pathname.slice(startIndex) || '/';
        }
        function getInvalidPathError(char, field, dest, path) {
          return (
            "Cannot include a '" +
            char +
            "' character in a manually specified `to." +
            field +
            '` field [' +
            JSON.stringify(path) +
            '].  Please separate it out to the `to.' +
            dest +
            '` field. Alternatively you may provide the full path as a string in <Link to="..."> and the router will parse it for you.'
          );
        }
        function getPathContributingMatches(matches) {
          return matches.filter(
            (match, index) => 0 === index || (match.route.path && match.route.path.length > 0),
          );
        }
        function router_resolveTo(toArg, routePathnames, locationPathname, isPathRelative) {
          let to;
          void 0 === isPathRelative && (isPathRelative = !1),
            'string' == typeof toArg
              ? (to = router_parsePath(toArg))
              : ((to = _extends({}, toArg)),
                invariant(
                  !to.pathname || !to.pathname.includes('?'),
                  getInvalidPathError('?', 'pathname', 'search', to),
                ),
                invariant(
                  !to.pathname || !to.pathname.includes('#'),
                  getInvalidPathError('#', 'pathname', 'hash', to),
                ),
                invariant(
                  !to.search || !to.search.includes('#'),
                  getInvalidPathError('#', 'search', 'hash', to),
                ));
          let from,
            isEmptyPath = '' === toArg || '' === to.pathname,
            toPathname = isEmptyPath ? '/' : to.pathname;
          if (isPathRelative || null == toPathname) from = locationPathname;
          else {
            let routePathnameIndex = routePathnames.length - 1;
            if (toPathname.startsWith('..')) {
              let toSegments = toPathname.split('/');
              for (; '..' === toSegments[0]; ) toSegments.shift(), (routePathnameIndex -= 1);
              to.pathname = toSegments.join('/');
            }
            from = routePathnameIndex >= 0 ? routePathnames[routePathnameIndex] : '/';
          }
          let path = (function resolvePath(to, fromPathname) {
              void 0 === fromPathname && (fromPathname = '/');
              let {
                  pathname: toPathname,
                  search = '',
                  hash = '',
                } = 'string' == typeof to ? router_parsePath(to) : to,
                pathname = toPathname
                  ? toPathname.startsWith('/')
                    ? toPathname
                    : (function resolvePathname(relativePath, fromPathname) {
                        let segments = fromPathname.replace(/\/+$/, '').split('/');
                        return (
                          relativePath.split('/').forEach((segment) => {
                            '..' === segment
                              ? segments.length > 1 && segments.pop()
                              : '.' !== segment && segments.push(segment);
                          }),
                          segments.length > 1 ? segments.join('/') : '/'
                        );
                      })(toPathname, fromPathname)
                  : fromPathname;
              return { pathname, search: normalizeSearch(search), hash: normalizeHash(hash) };
            })(to, from),
            hasExplicitTrailingSlash = toPathname && '/' !== toPathname && toPathname.endsWith('/'),
            hasCurrentTrailingSlash =
              (isEmptyPath || '.' === toPathname) && locationPathname.endsWith('/');
          return (
            path.pathname.endsWith('/') ||
              (!hasExplicitTrailingSlash && !hasCurrentTrailingSlash) ||
              (path.pathname += '/'),
            path
          );
        }
        const router_joinPaths = (paths) => paths.join('/').replace(/\/\/+/g, '/'),
          normalizeSearch = (search) =>
            search && '?' !== search ? (search.startsWith('?') ? search : '?' + search) : '',
          normalizeHash = (hash) =>
            hash && '#' !== hash ? (hash.startsWith('#') ? hash : '#' + hash) : '';
        Error;
        const validMutationMethodsArr = ['post', 'put', 'patch', 'delete'],
          validRequestMethodsArr =
            (new Set(validMutationMethodsArr), ['get', ...validMutationMethodsArr]);
        new Set(validRequestMethodsArr), new Set([301, 302, 303, 307, 308]), new Set([307, 308]);
        Symbol('deferred');
        function dist_extends() {
          return (
            (dist_extends = Object.assign
              ? Object.assign.bind()
              : function (target) {
                  for (var i = 1; i < arguments.length; i++) {
                    var source = arguments[i];
                    for (var key in source)
                      Object.prototype.hasOwnProperty.call(source, key) &&
                        (target[key] = source[key]);
                  }
                  return target;
                }),
            dist_extends.apply(this, arguments)
          );
        }
        const DataRouterContext = react.createContext(null);
        const NavigationContext = react.createContext(null);
        const LocationContext = react.createContext(null);
        const RouteContext = react.createContext({ outlet: null, matches: [], isDataRoute: !1 });
        function useInRouterContext() {
          return null != react.useContext(LocationContext);
        }
        function useLocation() {
          return useInRouterContext() || invariant(!1), react.useContext(LocationContext).location;
        }
        function useIsomorphicLayoutEffect(cb) {
          react.useContext(NavigationContext).static || react.useLayoutEffect(cb);
        }
        function useNavigate() {
          let { isDataRoute } = react.useContext(RouteContext);
          return isDataRoute
            ? (function useNavigateStable() {
                let { router } = useDataRouterContext(DataRouterHook.UseNavigateStable),
                  id = useCurrentRouteId(DataRouterStateHook.UseNavigateStable),
                  activeRef = react.useRef(!1);
                return (
                  useIsomorphicLayoutEffect(() => {
                    activeRef.current = !0;
                  }),
                  react.useCallback(
                    function (to, options) {
                      void 0 === options && (options = {}),
                        activeRef.current &&
                          ('number' == typeof to
                            ? router.navigate(to)
                            : router.navigate(to, dist_extends({ fromRouteId: id }, options)));
                    },
                    [router, id],
                  )
                );
              })()
            : (function useNavigateUnstable() {
                useInRouterContext() || invariant(!1);
                let dataRouterContext = react.useContext(DataRouterContext),
                  { basename, navigator } = react.useContext(NavigationContext),
                  { matches } = react.useContext(RouteContext),
                  { pathname: locationPathname } = useLocation(),
                  routePathnamesJson = JSON.stringify(
                    getPathContributingMatches(matches).map((match) => match.pathnameBase),
                  ),
                  activeRef = react.useRef(!1);
                return (
                  useIsomorphicLayoutEffect(() => {
                    activeRef.current = !0;
                  }),
                  react.useCallback(
                    function (to, options) {
                      if ((void 0 === options && (options = {}), !activeRef.current)) return;
                      if ('number' == typeof to) return void navigator.go(to);
                      let path = router_resolveTo(
                        to,
                        JSON.parse(routePathnamesJson),
                        locationPathname,
                        'path' === options.relative,
                      );
                      null == dataRouterContext &&
                        '/' !== basename &&
                        (path.pathname =
                          '/' === path.pathname
                            ? basename
                            : router_joinPaths([basename, path.pathname])),
                        (options.replace ? navigator.replace : navigator.push)(
                          path,
                          options.state,
                          options,
                        );
                    },
                    [basename, navigator, routePathnamesJson, locationPathname, dataRouterContext],
                  )
                );
              })();
        }
        react.Component;
        var DataRouterHook, DataRouterStateHook;
        function useDataRouterContext(hookName) {
          let ctx = react.useContext(DataRouterContext);
          return ctx || invariant(!1), ctx;
        }
        function useCurrentRouteId(hookName) {
          let route = (function useRouteContext(hookName) {
              let route = react.useContext(RouteContext);
              return route || invariant(!1), route;
            })(),
            thisRoute = route.matches[route.matches.length - 1];
          return thisRoute.route.id || invariant(!1), thisRoute.route.id;
        }
        !(function (DataRouterHook) {
          (DataRouterHook.UseBlocker = 'useBlocker'),
            (DataRouterHook.UseRevalidator = 'useRevalidator'),
            (DataRouterHook.UseNavigateStable = 'useNavigate');
        })(DataRouterHook || (DataRouterHook = {})),
          (function (DataRouterStateHook) {
            (DataRouterStateHook.UseBlocker = 'useBlocker'),
              (DataRouterStateHook.UseLoaderData = 'useLoaderData'),
              (DataRouterStateHook.UseActionData = 'useActionData'),
              (DataRouterStateHook.UseRouteError = 'useRouteError'),
              (DataRouterStateHook.UseNavigation = 'useNavigation'),
              (DataRouterStateHook.UseRouteLoaderData = 'useRouteLoaderData'),
              (DataRouterStateHook.UseMatches = 'useMatches'),
              (DataRouterStateHook.UseRevalidator = 'useRevalidator'),
              (DataRouterStateHook.UseNavigateStable = 'useNavigate'),
              (DataRouterStateHook.UseRouteId = 'useRouteId');
          })(DataRouterStateHook || (DataRouterStateHook = {}));
        const startTransitionImpl = react_namespaceObject.startTransition;
        function MemoryRouter(_ref3) {
          let { basename, children, initialEntries, initialIndex, future } = _ref3,
            historyRef = react.useRef();
          null == historyRef.current &&
            (historyRef.current = (function router_createMemoryHistory(options) {
              void 0 === options && (options = {});
              let entries,
                { initialEntries = ['/'], initialIndex, v5Compat = !1 } = options;
              entries = initialEntries.map((entry, index) =>
                createMemoryLocation(
                  entry,
                  'string' == typeof entry ? null : entry.state,
                  0 === index ? 'default' : void 0,
                ),
              );
              let index = clampIndex(null == initialIndex ? entries.length - 1 : initialIndex),
                action = router_Action.Pop,
                listener = null;
              function clampIndex(n) {
                return Math.min(Math.max(n, 0), entries.length - 1);
              }
              function getCurrentLocation() {
                return entries[index];
              }
              function createMemoryLocation(to, state, key) {
                void 0 === state && (state = null);
                let location = createLocation(
                  entries ? getCurrentLocation().pathname : '/',
                  to,
                  state,
                  key,
                );
                return (
                  warning(
                    '/' === location.pathname.charAt(0),
                    'relative pathnames are not supported in memory history: ' + JSON.stringify(to),
                  ),
                  location
                );
              }
              function createHref(to) {
                return 'string' == typeof to ? to : createPath(to);
              }
              return {
                get index() {
                  return index;
                },
                get action() {
                  return action;
                },
                get location() {
                  return getCurrentLocation();
                },
                createHref,
                createURL: (to) => new URL(createHref(to), 'http://localhost'),
                encodeLocation(to) {
                  let path = 'string' == typeof to ? router_parsePath(to) : to;
                  return {
                    pathname: path.pathname || '',
                    search: path.search || '',
                    hash: path.hash || '',
                  };
                },
                push(to, state) {
                  action = router_Action.Push;
                  let nextLocation = createMemoryLocation(to, state);
                  (index += 1),
                    entries.splice(index, entries.length, nextLocation),
                    v5Compat && listener && listener({ action, location: nextLocation, delta: 1 });
                },
                replace(to, state) {
                  action = router_Action.Replace;
                  let nextLocation = createMemoryLocation(to, state);
                  (entries[index] = nextLocation),
                    v5Compat && listener && listener({ action, location: nextLocation, delta: 0 });
                },
                go(delta) {
                  action = router_Action.Pop;
                  let nextIndex = clampIndex(index + delta),
                    nextLocation = entries[nextIndex];
                  (index = nextIndex),
                    listener && listener({ action, location: nextLocation, delta });
                },
                listen: (fn) => (
                  (listener = fn),
                  () => {
                    listener = null;
                  }
                ),
              };
            })({ initialEntries, initialIndex, v5Compat: !0 }));
          let history = historyRef.current,
            [state, setStateImpl] = react.useState({
              action: history.action,
              location: history.location,
            }),
            { v7_startTransition } = future || {},
            setState = react.useCallback(
              (newState) => {
                v7_startTransition && startTransitionImpl
                  ? startTransitionImpl(() => setStateImpl(newState))
                  : setStateImpl(newState);
              },
              [setStateImpl, v7_startTransition],
            );
          return (
            react.useLayoutEffect(() => history.listen(setState), [history, setState]),
            react.createElement(Router, {
              basename,
              children,
              location: state.location,
              navigationType: state.action,
              navigator: history,
            })
          );
        }
        function Router(_ref5) {
          let {
            basename: basenameProp = '/',
            children = null,
            location: locationProp,
            navigationType = router_Action.Pop,
            navigator,
            static: staticProp = !1,
          } = _ref5;
          useInRouterContext() && invariant(!1);
          let basename = basenameProp.replace(/^\/*/, '/'),
            navigationContext = react.useMemo(
              () => ({ basename, navigator, static: staticProp }),
              [basename, navigator, staticProp],
            );
          'string' == typeof locationProp && (locationProp = router_parsePath(locationProp));
          let {
              pathname = '/',
              search = '',
              hash = '',
              state = null,
              key = 'default',
            } = locationProp,
            locationContext = react.useMemo(() => {
              let trailingPathname = router_stripBasename(pathname, basename);
              return null == trailingPathname
                ? null
                : {
                    location: { pathname: trailingPathname, search, hash, state, key },
                    navigationType,
                  };
            }, [basename, pathname, search, hash, state, key, navigationType]);
          return null == locationContext
            ? null
            : react.createElement(
                NavigationContext.Provider,
                { value: navigationContext },
                react.createElement(LocationContext.Provider, { children, value: locationContext }),
              );
        }
        var AwaitRenderStatus;
        !(function (AwaitRenderStatus) {
          (AwaitRenderStatus[(AwaitRenderStatus.pending = 0)] = 'pending'),
            (AwaitRenderStatus[(AwaitRenderStatus.success = 1)] = 'success'),
            (AwaitRenderStatus[(AwaitRenderStatus.error = 2)] = 'error');
        })(AwaitRenderStatus || (AwaitRenderStatus = {}));
        new Promise(() => {});
        react.Component;
      },
  },
]);
