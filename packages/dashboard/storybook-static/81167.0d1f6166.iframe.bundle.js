(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [81167],
  {
    '../api-client/dist/lib/index.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __createBinding =
          (this && this.__createBinding) ||
          (Object.create
            ? function (o, m, k, k2) {
                void 0 === k2 && (k2 = k),
                  Object.defineProperty(o, k2, {
                    enumerable: !0,
                    get: function () {
                      return m[k];
                    },
                  });
              }
            : function (o, m, k, k2) {
                void 0 === k2 && (k2 = k), (o[k2] = m[k]);
              }),
        __exportStar =
          (this && this.__exportStar) ||
          function (m, exports) {
            for (var p in m)
              'default' === p ||
                Object.prototype.hasOwnProperty.call(exports, p) ||
                __createBinding(exports, m, p);
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          },
        __spreadArray =
          (this && this.__spreadArray) ||
          function (to, from, pack) {
            if (pack || 2 === arguments.length)
              for (var ar, i = 0, l = from.length; i < l; i++)
                (!ar && i in from) ||
                  (ar || (ar = Array.prototype.slice.call(from, 0, i)), (ar[i] = from[i]));
            return to.concat(ar || Array.prototype.slice.call(from));
          },
        __importDefault =
          (this && this.__importDefault) ||
          function (mod) {
            return mod && mod.__esModule ? mod : { default: mod };
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.SioClient = void 0);
      var debug_1 = __importDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/debug@4.3.4_supports-color@5.5.0/node_modules/debug/src/browser.js',
          ),
        ),
        socket_io_client_1 = __webpack_require__(
          '../../node_modules/.pnpm/socket.io-client@3.1.3/node_modules/socket.io-client/build/index.js',
        ),
        debug = (0, debug_1.default)('rmf-client'),
        SioClient = (function () {
          function SioClient() {
            for (var args = [], _i = 0; _i < arguments.length; _i++) args[_i] = arguments[_i];
            (this._subscriptions = {}),
              (this.sio = socket_io_client_1.io.apply(void 0, __spreadArray([], __read(args), !1)));
          }
          return (
            (SioClient.prototype.subscribe = function (room, listener) {
              var subs = this._subscriptions[room] || 0;
              return (
                0 === subs
                  ? (this.sio.emit('subscribe', { room }), debug('subscribed to ' + room))
                  : debug('reusing previous subscription to ' + room),
                this.sio.on(room, listener),
                (this._subscriptions[room] = subs + 1),
                { room, listener }
              );
            }),
            (SioClient.prototype.unsubscribe = function (sub) {
              var subCount = this._subscriptions[sub.room] || 0;
              subCount ||
                debug('tried to unsubscribe from ' + sub.room + ', but no subscriptions exist'),
                subCount <= 1
                  ? (this.sio.emit('unsubscribe', { room: sub.room }),
                    delete this._subscriptions[sub.room],
                    debug('unsubscribed to ' + sub.room))
                  : ((this._subscriptions[sub.room] = subCount - 1),
                    debug(
                      'skipping unsubscribe to ' +
                        sub.room +
                        ' because there are still ' +
                        (subCount - 1) +
                        ' subscribers',
                    )),
                this.sio.off(sub.room, sub.listener);
            }),
            (SioClient.prototype.subscribeBeaconState = function (listener) {
              return this.subscribe('/beacons', listener);
            }),
            (SioClient.prototype.subscribeBuildingMap = function (listener) {
              return this.subscribe('/building_map', listener);
            }),
            (SioClient.prototype.subscribeDoorState = function (doorName, listener) {
              return this.subscribe('/doors/' + doorName + '/state', listener);
            }),
            (SioClient.prototype.subscribeDoorHealth = function (doorName, listener) {
              return this.subscribe('/doors/' + doorName + '/health', listener);
            }),
            (SioClient.prototype.subscribeLiftState = function (liftName, listener) {
              return this.subscribe('/lifts/' + liftName + '/state', listener);
            }),
            (SioClient.prototype.subscribeLiftHealth = function (liftName, listener) {
              return this.subscribe('/doors/' + liftName + '/health', listener);
            }),
            (SioClient.prototype.subscribeDispenserState = function (guid, listener) {
              return this.subscribe('/dispensers/' + guid + '/state', listener);
            }),
            (SioClient.prototype.subscribeDispenserHealth = function (guid, listener) {
              return this.subscribe('/dispensers/' + guid + '/health', listener);
            }),
            (SioClient.prototype.subscribeIngestorState = function (guid, listener) {
              return this.subscribe('/ingestors/' + guid + '/state', listener);
            }),
            (SioClient.prototype.subscribeIngestorHealth = function (guid, listener) {
              return this.subscribe('/ingestors/' + guid + '/health', listener);
            }),
            (SioClient.prototype.subscribeFleetState = function (name, listener) {
              return this.subscribe('/fleets/' + name + '/state', listener);
            }),
            (SioClient.prototype.subscribeTaskState = function (taskId, listener) {
              return this.subscribe('/tasks/' + taskId + '/state', listener);
            }),
            (SioClient.prototype.subscribeTaskLogs = function (taskId, listener) {
              return this.subscribe('/tasks/' + taskId + '/log', listener);
            }),
            (SioClient.prototype.subscribeAlerts = function (listener) {
              return this.subscribe('/alerts', listener);
            }),
            SioClient
          );
        })();
      (exports.SioClient = SioClient),
        __exportStar(__webpack_require__('../api-client/dist/lib/openapi/index.js'), exports);
    },
    '../api-client/dist/lib/openapi/api.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var extendStatics,
        __extends =
          (this && this.__extends) ||
          ((extendStatics = function (d, b) {
            return (
              (extendStatics =
                Object.setPrototypeOf ||
                ({ __proto__: [] } instanceof Array &&
                  function (d, b) {
                    d.__proto__ = b;
                  }) ||
                function (d, b) {
                  for (var p in b) Object.prototype.hasOwnProperty.call(b, p) && (d[p] = b[p]);
                }),
              extendStatics(d, b)
            );
          }),
          function (d, b) {
            if ('function' != typeof b && null !== b)
              throw new TypeError(
                'Class extends value ' + String(b) + ' is not a constructor or null',
              );
            function __() {
              this.constructor = d;
            }
            extendStatics(d, b),
              (d.prototype =
                null === b ? Object.create(b) : ((__.prototype = b.prototype), new __()));
          }),
        __assign =
          (this && this.__assign) ||
          function () {
            return (
              (__assign =
                Object.assign ||
                function (t) {
                  for (var s, i = 1, n = arguments.length; i < n; i++)
                    for (var p in (s = arguments[i]))
                      Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                  return t;
                }),
              __assign.apply(this, arguments)
            );
          },
        __awaiter =
          (this && this.__awaiter) ||
          function (thisArg, _arguments, P, generator) {
            return new (P || (P = Promise))(function (resolve, reject) {
              function fulfilled(value) {
                try {
                  step(generator.next(value));
                } catch (e) {
                  reject(e);
                }
              }
              function rejected(value) {
                try {
                  step(generator.throw(value));
                } catch (e) {
                  reject(e);
                }
              }
              function step(result) {
                result.done
                  ? resolve(result.value)
                  : (function adopt(value) {
                      return value instanceof P
                        ? value
                        : new P(function (resolve) {
                            resolve(value);
                          });
                    })(result.value).then(fulfilled, rejected);
              }
              step((generator = generator.apply(thisArg, _arguments || [])).next());
            });
          },
        __generator =
          (this && this.__generator) ||
          function (thisArg, body) {
            var f,
              y,
              t,
              g,
              _ = {
                label: 0,
                sent: function () {
                  if (1 & t[0]) throw t[1];
                  return t[1];
                },
                trys: [],
                ops: [],
              };
            return (
              (g = { next: verb(0), throw: verb(1), return: verb(2) }),
              'function' == typeof Symbol &&
                (g[Symbol.iterator] = function () {
                  return this;
                }),
              g
            );
            function verb(n) {
              return function (v) {
                return (function step(op) {
                  if (f) throw new TypeError('Generator is already executing.');
                  for (; _; )
                    try {
                      if (
                        ((f = 1),
                        y &&
                          (t =
                            2 & op[0]
                              ? y.return
                              : op[0]
                                ? y.throw || ((t = y.return) && t.call(y), 0)
                                : y.next) &&
                          !(t = t.call(y, op[1])).done)
                      )
                        return t;
                      switch (((y = 0), t && (op = [2 & op[0], t.value]), op[0])) {
                        case 0:
                        case 1:
                          t = op;
                          break;
                        case 4:
                          return _.label++, { value: op[1], done: !1 };
                        case 5:
                          _.label++, (y = op[1]), (op = [0]);
                          continue;
                        case 7:
                          (op = _.ops.pop()), _.trys.pop();
                          continue;
                        default:
                          if (
                            !((t = _.trys),
                            (t = t.length > 0 && t[t.length - 1]) || (6 !== op[0] && 2 !== op[0]))
                          ) {
                            _ = 0;
                            continue;
                          }
                          if (3 === op[0] && (!t || (op[1] > t[0] && op[1] < t[3]))) {
                            _.label = op[1];
                            break;
                          }
                          if (6 === op[0] && _.label < t[1]) {
                            (_.label = t[1]), (t = op);
                            break;
                          }
                          if (t && _.label < t[2]) {
                            (_.label = t[2]), _.ops.push(op);
                            break;
                          }
                          t[2] && _.ops.pop(), _.trys.pop();
                          continue;
                      }
                      op = body.call(thisArg, _);
                    } catch (e) {
                      (op = [6, e]), (y = 0);
                    } finally {
                      f = t = 0;
                    }
                  if (5 & op[0]) throw op[1];
                  return { value: op[0] ? op[1] : void 0, done: !0 };
                })([n, v]);
              };
            }
          },
        __importDefault =
          (this && this.__importDefault) ||
          function (mod) {
            return mod && mod.__esModule ? mod : { default: mod };
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.TasksApi =
          exports.TasksApiFactory =
          exports.TasksApiFp =
          exports.TasksApiAxiosParamCreator =
          exports.LiftsApi =
          exports.LiftsApiFactory =
          exports.LiftsApiFp =
          exports.LiftsApiAxiosParamCreator =
          exports.IngestorsApi =
          exports.IngestorsApiFactory =
          exports.IngestorsApiFp =
          exports.IngestorsApiAxiosParamCreator =
          exports.FleetsApi =
          exports.FleetsApiFactory =
          exports.FleetsApiFp =
          exports.FleetsApiAxiosParamCreator =
          exports.DoorsApi =
          exports.DoorsApiFactory =
          exports.DoorsApiFp =
          exports.DoorsApiAxiosParamCreator =
          exports.DispensersApi =
          exports.DispensersApiFactory =
          exports.DispensersApiFp =
          exports.DispensersApiAxiosParamCreator =
          exports.DefaultApi =
          exports.DefaultApiFactory =
          exports.DefaultApiFp =
          exports.DefaultApiAxiosParamCreator =
          exports.BuildingApi =
          exports.BuildingApiFactory =
          exports.BuildingApiFp =
          exports.BuildingApiAxiosParamCreator =
          exports.BeaconsApi =
          exports.BeaconsApiFactory =
          exports.BeaconsApiFp =
          exports.BeaconsApiAxiosParamCreator =
          exports.AlertsApi =
          exports.AlertsApiFactory =
          exports.AlertsApiFp =
          exports.AlertsApiAxiosParamCreator =
          exports.AdminApi =
          exports.AdminApiFactory =
          exports.AdminApiFp =
          exports.AdminApiAxiosParamCreator =
          exports.Tier =
          exports.Status2 =
          exports.Period =
          exports.HealthStatus =
          exports.ApiServerModelsRmfApiTaskStateStatus =
          exports.ApiServerModelsRmfApiRobotStateStatus =
            void 0);
      var axios_1 = __importDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/axios@1.6.8_debug@4.3.4/node_modules/axios/dist/browser/axios.cjs',
          ),
        ),
        common_1 = __webpack_require__('../api-client/dist/lib/openapi/common.js'),
        base_1 = __webpack_require__('../api-client/dist/lib/openapi/base.js');
      (exports.ApiServerModelsRmfApiRobotStateStatus = {
        Uninitialized: 'uninitialized',
        Offline: 'offline',
        Shutdown: 'shutdown',
        Idle: 'idle',
        Charging: 'charging',
        Working: 'working',
        Error: 'error',
      }),
        (exports.ApiServerModelsRmfApiTaskStateStatus = {
          Uninitialized: 'uninitialized',
          Blocked: 'blocked',
          Error: 'error',
          Failed: 'failed',
          Queued: 'queued',
          Standby: 'standby',
          Underway: 'underway',
          Delayed: 'delayed',
          Skipped: 'skipped',
          Canceled: 'canceled',
          Killed: 'killed',
          Completed: 'completed',
        }),
        (exports.HealthStatus = { Healthy: 'Healthy', Unhealthy: 'Unhealthy', Dead: 'Dead' }),
        (exports.Period = {
          Monday: 'monday',
          Tuesday: 'tuesday',
          Wednesday: 'wednesday',
          Thursday: 'thursday',
          Friday: 'friday',
          Saturday: 'saturday',
          Sunday: 'sunday',
          Day: 'day',
          Hour: 'hour',
          Minute: 'minute',
        }),
        (exports.Status2 = {
          Queued: 'queued',
          Selected: 'selected',
          Dispatched: 'dispatched',
          FailedToAssign: 'failed_to_assign',
          CanceledInFlight: 'canceled_in_flight',
        }),
        (exports.Tier = {
          Uninitialized: 'uninitialized',
          Info: 'info',
          Warning: 'warning',
          Error: 'error',
        });
      exports.AdminApiAxiosParamCreator = function (configuration) {
        var _this = this;
        return {
          addRolePermissionAdminRolesRolePermissionsPost: function (
            role,
            permission,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'addRolePermissionAdminRolesRolePermissionsPost',
                      'role',
                      role,
                    ),
                    (0, common_1.assertParamExists)(
                      'addRolePermissionAdminRolesRolePermissionsPost',
                      'permission',
                      permission,
                    ),
                    (localVarPath = '/admin/roles/{role}/permissions'.replace(
                      '{role}',
                      encodeURIComponent(String(role)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      permission,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          addUserRoleAdminUsersUsernameRolesPost: function (
            username,
            postRoles,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'addUserRoleAdminUsersUsernameRolesPost',
                      'username',
                      username,
                    ),
                    (0, common_1.assertParamExists)(
                      'addUserRoleAdminUsersUsernameRolesPost',
                      'postRoles',
                      postRoles,
                    ),
                    (localVarPath = '/admin/users/{username}/roles'.replace(
                      '{username}',
                      encodeURIComponent(String(username)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      postRoles,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          createRoleAdminRolesPost: function (postRoles, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'createRoleAdminRolesPost',
                      'postRoles',
                      postRoles,
                    ),
                    '/admin/roles',
                    (localVarUrlObj = new URL('/admin/roles', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      postRoles,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          createUserAdminUsersPost: function (postUsers, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'createUserAdminUsersPost',
                      'postUsers',
                      postUsers,
                    ),
                    '/admin/users',
                    (localVarUrlObj = new URL('/admin/users', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      postUsers,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          deleteRoleAdminRolesRoleDelete: function (role, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)('deleteRoleAdminRolesRoleDelete', 'role', role),
                    (localVarPath = '/admin/roles/{role}'.replace(
                      '{role}',
                      encodeURIComponent(String(role)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'DELETE' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          deleteUserAdminUsersUsernameDelete: function (username, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'deleteUserAdminUsersUsernameDelete',
                      'username',
                      username,
                    ),
                    (localVarPath = '/admin/users/{username}'.replace(
                      '{username}',
                      encodeURIComponent(String(username)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'DELETE' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          deleteUserRoleAdminUsersUsernameRolesRoleDelete: function (
            username,
            role,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'deleteUserRoleAdminUsersUsernameRolesRoleDelete',
                      'username',
                      username,
                    ),
                    (0, common_1.assertParamExists)(
                      'deleteUserRoleAdminUsersUsernameRolesRoleDelete',
                      'role',
                      role,
                    ),
                    (localVarPath = '/admin/users/{username}/roles/{role}'
                      .replace('{username}', encodeURIComponent(String(username)))
                      .replace('{role}', encodeURIComponent(String(role)))),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'DELETE' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getRolePermissionsAdminRolesRolePermissionsGet: function (role, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'getRolePermissionsAdminRolesRolePermissionsGet',
                      'role',
                      role,
                    ),
                    (localVarPath = '/admin/roles/{role}/permissions'.replace(
                      '{role}',
                      encodeURIComponent(String(role)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getRolesAdminRolesGet: function (authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    '/admin/roles',
                    (localVarUrlObj = new URL('/admin/roles', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getUserAdminUsersUsernameGet: function (username, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'getUserAdminUsersUsernameGet',
                      'username',
                      username,
                    ),
                    (localVarPath = '/admin/users/{username}'.replace(
                      '{username}',
                      encodeURIComponent(String(username)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getUsersAdminUsersGet: function (
            username,
            isAdmin,
            limit,
            offset,
            orderBy,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    '/admin/users',
                    (localVarUrlObj = new URL('/admin/users', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    void 0 !== username && (localVarQueryParameter.username = username),
                    void 0 !== isAdmin && (localVarQueryParameter.is_admin = isAdmin),
                    void 0 !== limit && (localVarQueryParameter.limit = limit),
                    void 0 !== offset && (localVarQueryParameter.offset = offset),
                    void 0 !== orderBy && (localVarQueryParameter.order_by = orderBy),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          makeAdminAdminUsersUsernameMakeAdminPost: function (
            username,
            postMakeAdmin,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'makeAdminAdminUsersUsernameMakeAdminPost',
                      'username',
                      username,
                    ),
                    (0, common_1.assertParamExists)(
                      'makeAdminAdminUsersUsernameMakeAdminPost',
                      'postMakeAdmin',
                      postMakeAdmin,
                    ),
                    (localVarPath = '/admin/users/{username}/make_admin'.replace(
                      '{username}',
                      encodeURIComponent(String(username)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      postMakeAdmin,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          removeRolePermissionAdminRolesRolePermissionsRemovePost: function (
            role,
            permission,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'removeRolePermissionAdminRolesRolePermissionsRemovePost',
                      'role',
                      role,
                    ),
                    (0, common_1.assertParamExists)(
                      'removeRolePermissionAdminRolesRolePermissionsRemovePost',
                      'permission',
                      permission,
                    ),
                    (localVarPath = '/admin/roles/{role}/permissions/remove'.replace(
                      '{role}',
                      encodeURIComponent(String(role)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      permission,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          setUserRolesAdminUsersUsernameRolesPut: function (
            username,
            postRoles,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'setUserRolesAdminUsersUsernameRolesPut',
                      'username',
                      username,
                    ),
                    (0, common_1.assertParamExists)(
                      'setUserRolesAdminUsersUsernameRolesPut',
                      'postRoles',
                      postRoles,
                    ),
                    (localVarPath = '/admin/users/{username}/roles'.replace(
                      '{username}',
                      encodeURIComponent(String(username)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'PUT' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      postRoles,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
        };
      };
      exports.AdminApiFp = function (configuration) {
        var localVarAxiosParamCreator = (0, exports.AdminApiAxiosParamCreator)(configuration);
        return {
          addRolePermissionAdminRolesRolePermissionsPost: function (
            role,
            permission,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.addRolePermissionAdminRolesRolePermissionsPost(
                        role,
                        permission,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'AdminApi.addRolePermissionAdminRolesRolePermissionsPost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          addUserRoleAdminUsersUsernameRolesPost: function (
            username,
            postRoles,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.addUserRoleAdminUsersUsernameRolesPost(
                        username,
                        postRoles,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'AdminApi.addUserRoleAdminUsersUsernameRolesPost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          createRoleAdminRolesPost: function (postRoles, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.createRoleAdminRolesPost(
                        postRoles,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap['AdminApi.createRoleAdminRolesPost']) ||
                            void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          createUserAdminUsersPost: function (postUsers, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.createUserAdminUsersPost(
                        postUsers,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap['AdminApi.createUserAdminUsersPost']) ||
                            void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          deleteRoleAdminRolesRoleDelete: function (role, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.deleteRoleAdminRolesRoleDelete(
                        role,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'AdminApi.deleteRoleAdminRolesRoleDelete'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          deleteUserAdminUsersUsernameDelete: function (username, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.deleteUserAdminUsersUsernameDelete(
                        username,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'AdminApi.deleteUserAdminUsersUsernameDelete'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          deleteUserRoleAdminUsersUsernameRolesRoleDelete: function (
            username,
            role,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.deleteUserRoleAdminUsersUsernameRolesRoleDelete(
                        username,
                        role,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'AdminApi.deleteUserRoleAdminUsersUsernameRolesRoleDelete'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getRolePermissionsAdminRolesRolePermissionsGet: function (role, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getRolePermissionsAdminRolesRolePermissionsGet(
                        role,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'AdminApi.getRolePermissionsAdminRolesRolePermissionsGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getRolesAdminRolesGet: function (authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getRolesAdminRolesGet(authorization, options),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b = base_1.operationServerMap['AdminApi.getRolesAdminRolesGet']) ||
                            void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getUserAdminUsersUsernameGet: function (username, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getUserAdminUsersUsernameGet(
                        username,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'AdminApi.getUserAdminUsersUsernameGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getUsersAdminUsersGet: function (
            username,
            isAdmin,
            limit,
            offset,
            orderBy,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getUsersAdminUsersGet(
                        username,
                        isAdmin,
                        limit,
                        offset,
                        orderBy,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b = base_1.operationServerMap['AdminApi.getUsersAdminUsersGet']) ||
                            void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          makeAdminAdminUsersUsernameMakeAdminPost: function (
            username,
            postMakeAdmin,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.makeAdminAdminUsersUsernameMakeAdminPost(
                        username,
                        postMakeAdmin,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'AdminApi.makeAdminAdminUsersUsernameMakeAdminPost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          removeRolePermissionAdminRolesRolePermissionsRemovePost: function (
            role,
            permission,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.removeRolePermissionAdminRolesRolePermissionsRemovePost(
                        role,
                        permission,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'AdminApi.removeRolePermissionAdminRolesRolePermissionsRemovePost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          setUserRolesAdminUsersUsernameRolesPut: function (
            username,
            postRoles,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.setUserRolesAdminUsersUsernameRolesPut(
                        username,
                        postRoles,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'AdminApi.setUserRolesAdminUsersUsernameRolesPut'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
        };
      };
      exports.AdminApiFactory = function (configuration, basePath, axios) {
        var localVarFp = (0, exports.AdminApiFp)(configuration);
        return {
          addRolePermissionAdminRolesRolePermissionsPost: function (
            role,
            permission,
            authorization,
            options,
          ) {
            return localVarFp
              .addRolePermissionAdminRolesRolePermissionsPost(
                role,
                permission,
                authorization,
                options,
              )
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          addUserRoleAdminUsersUsernameRolesPost: function (
            username,
            postRoles,
            authorization,
            options,
          ) {
            return localVarFp
              .addUserRoleAdminUsersUsernameRolesPost(username, postRoles, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          createRoleAdminRolesPost: function (postRoles, authorization, options) {
            return localVarFp
              .createRoleAdminRolesPost(postRoles, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          createUserAdminUsersPost: function (postUsers, authorization, options) {
            return localVarFp
              .createUserAdminUsersPost(postUsers, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          deleteRoleAdminRolesRoleDelete: function (role, authorization, options) {
            return localVarFp
              .deleteRoleAdminRolesRoleDelete(role, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          deleteUserAdminUsersUsernameDelete: function (username, authorization, options) {
            return localVarFp
              .deleteUserAdminUsersUsernameDelete(username, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          deleteUserRoleAdminUsersUsernameRolesRoleDelete: function (
            username,
            role,
            authorization,
            options,
          ) {
            return localVarFp
              .deleteUserRoleAdminUsersUsernameRolesRoleDelete(
                username,
                role,
                authorization,
                options,
              )
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getRolePermissionsAdminRolesRolePermissionsGet: function (role, authorization, options) {
            return localVarFp
              .getRolePermissionsAdminRolesRolePermissionsGet(role, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getRolesAdminRolesGet: function (authorization, options) {
            return localVarFp
              .getRolesAdminRolesGet(authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getUserAdminUsersUsernameGet: function (username, authorization, options) {
            return localVarFp
              .getUserAdminUsersUsernameGet(username, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getUsersAdminUsersGet: function (
            username,
            isAdmin,
            limit,
            offset,
            orderBy,
            authorization,
            options,
          ) {
            return localVarFp
              .getUsersAdminUsersGet(
                username,
                isAdmin,
                limit,
                offset,
                orderBy,
                authorization,
                options,
              )
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          makeAdminAdminUsersUsernameMakeAdminPost: function (
            username,
            postMakeAdmin,
            authorization,
            options,
          ) {
            return localVarFp
              .makeAdminAdminUsersUsernameMakeAdminPost(
                username,
                postMakeAdmin,
                authorization,
                options,
              )
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          removeRolePermissionAdminRolesRolePermissionsRemovePost: function (
            role,
            permission,
            authorization,
            options,
          ) {
            return localVarFp
              .removeRolePermissionAdminRolesRolePermissionsRemovePost(
                role,
                permission,
                authorization,
                options,
              )
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          setUserRolesAdminUsersUsernameRolesPut: function (
            username,
            postRoles,
            authorization,
            options,
          ) {
            return localVarFp
              .setUserRolesAdminUsersUsernameRolesPut(username, postRoles, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
        };
      };
      var AdminApi = (function (_super) {
        function AdminApi() {
          return (null !== _super && _super.apply(this, arguments)) || this;
        }
        return (
          __extends(AdminApi, _super),
          (AdminApi.prototype.addRolePermissionAdminRolesRolePermissionsPost = function (
            role,
            permission,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.AdminApiFp)(this.configuration)
              .addRolePermissionAdminRolesRolePermissionsPost(
                role,
                permission,
                authorization,
                options,
              )
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (AdminApi.prototype.addUserRoleAdminUsersUsernameRolesPost = function (
            username,
            postRoles,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.AdminApiFp)(this.configuration)
              .addUserRoleAdminUsersUsernameRolesPost(username, postRoles, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (AdminApi.prototype.createRoleAdminRolesPost = function (
            postRoles,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.AdminApiFp)(this.configuration)
              .createRoleAdminRolesPost(postRoles, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (AdminApi.prototype.createUserAdminUsersPost = function (
            postUsers,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.AdminApiFp)(this.configuration)
              .createUserAdminUsersPost(postUsers, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (AdminApi.prototype.deleteRoleAdminRolesRoleDelete = function (
            role,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.AdminApiFp)(this.configuration)
              .deleteRoleAdminRolesRoleDelete(role, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (AdminApi.prototype.deleteUserAdminUsersUsernameDelete = function (
            username,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.AdminApiFp)(this.configuration)
              .deleteUserAdminUsersUsernameDelete(username, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (AdminApi.prototype.deleteUserRoleAdminUsersUsernameRolesRoleDelete = function (
            username,
            role,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.AdminApiFp)(this.configuration)
              .deleteUserRoleAdminUsersUsernameRolesRoleDelete(
                username,
                role,
                authorization,
                options,
              )
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (AdminApi.prototype.getRolePermissionsAdminRolesRolePermissionsGet = function (
            role,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.AdminApiFp)(this.configuration)
              .getRolePermissionsAdminRolesRolePermissionsGet(role, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (AdminApi.prototype.getRolesAdminRolesGet = function (authorization, options) {
            var _this = this;
            return (0, exports.AdminApiFp)(this.configuration)
              .getRolesAdminRolesGet(authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (AdminApi.prototype.getUserAdminUsersUsernameGet = function (
            username,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.AdminApiFp)(this.configuration)
              .getUserAdminUsersUsernameGet(username, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (AdminApi.prototype.getUsersAdminUsersGet = function (
            username,
            isAdmin,
            limit,
            offset,
            orderBy,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.AdminApiFp)(this.configuration)
              .getUsersAdminUsersGet(
                username,
                isAdmin,
                limit,
                offset,
                orderBy,
                authorization,
                options,
              )
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (AdminApi.prototype.makeAdminAdminUsersUsernameMakeAdminPost = function (
            username,
            postMakeAdmin,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.AdminApiFp)(this.configuration)
              .makeAdminAdminUsersUsernameMakeAdminPost(
                username,
                postMakeAdmin,
                authorization,
                options,
              )
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (AdminApi.prototype.removeRolePermissionAdminRolesRolePermissionsRemovePost = function (
            role,
            permission,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.AdminApiFp)(this.configuration)
              .removeRolePermissionAdminRolesRolePermissionsRemovePost(
                role,
                permission,
                authorization,
                options,
              )
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (AdminApi.prototype.setUserRolesAdminUsersUsernameRolesPut = function (
            username,
            postRoles,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.AdminApiFp)(this.configuration)
              .setUserRolesAdminUsersUsernameRolesPut(username, postRoles, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          AdminApi
        );
      })(base_1.BaseAPI);
      exports.AdminApi = AdminApi;
      exports.AlertsApiAxiosParamCreator = function (configuration) {
        var _this = this;
        return {
          acknowledgeAlertAlertsAlertIdPost: function (alertId, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'acknowledgeAlertAlertsAlertIdPost',
                      'alertId',
                      alertId,
                    ),
                    (localVarPath = '/alerts/{alert_id}'.replace(
                      '{alert_id}',
                      encodeURIComponent(String(alertId)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          createAlertAlertsPost: function (alertId, category, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)('createAlertAlertsPost', 'alertId', alertId),
                    (0, common_1.assertParamExists)('createAlertAlertsPost', 'category', category),
                    '/alerts',
                    (localVarUrlObj = new URL('/alerts', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    void 0 !== alertId && (localVarQueryParameter.alert_id = alertId),
                    void 0 !== category && (localVarQueryParameter.category = category),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getAlertAlertsAlertIdGet: function (alertId, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)('getAlertAlertsAlertIdGet', 'alertId', alertId),
                    (localVarPath = '/alerts/{alert_id}'.replace(
                      '{alert_id}',
                      encodeURIComponent(String(alertId)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getAlertsAlertsGet: function (authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    '/alerts',
                    (localVarUrlObj = new URL('/alerts', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
        };
      };
      exports.AlertsApiFp = function (configuration) {
        var localVarAxiosParamCreator = (0, exports.AlertsApiAxiosParamCreator)(configuration);
        return {
          acknowledgeAlertAlertsAlertIdPost: function (alertId, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.acknowledgeAlertAlertsAlertIdPost(
                        alertId,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'AlertsApi.acknowledgeAlertAlertsAlertIdPost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          createAlertAlertsPost: function (alertId, category, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.createAlertAlertsPost(
                        alertId,
                        category,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b = base_1.operationServerMap['AlertsApi.createAlertAlertsPost']) ||
                            void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getAlertAlertsAlertIdGet: function (alertId, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getAlertAlertsAlertIdGet(
                        alertId,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap['AlertsApi.getAlertAlertsAlertIdGet']) ||
                            void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getAlertsAlertsGet: function (authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getAlertsAlertsGet(authorization, options),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b = base_1.operationServerMap['AlertsApi.getAlertsAlertsGet']) ||
                            void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
        };
      };
      exports.AlertsApiFactory = function (configuration, basePath, axios) {
        var localVarFp = (0, exports.AlertsApiFp)(configuration);
        return {
          acknowledgeAlertAlertsAlertIdPost: function (alertId, authorization, options) {
            return localVarFp
              .acknowledgeAlertAlertsAlertIdPost(alertId, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          createAlertAlertsPost: function (alertId, category, authorization, options) {
            return localVarFp
              .createAlertAlertsPost(alertId, category, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getAlertAlertsAlertIdGet: function (alertId, authorization, options) {
            return localVarFp
              .getAlertAlertsAlertIdGet(alertId, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getAlertsAlertsGet: function (authorization, options) {
            return localVarFp.getAlertsAlertsGet(authorization, options).then(function (request) {
              return request(axios, basePath);
            });
          },
        };
      };
      var AlertsApi = (function (_super) {
        function AlertsApi() {
          return (null !== _super && _super.apply(this, arguments)) || this;
        }
        return (
          __extends(AlertsApi, _super),
          (AlertsApi.prototype.acknowledgeAlertAlertsAlertIdPost = function (
            alertId,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.AlertsApiFp)(this.configuration)
              .acknowledgeAlertAlertsAlertIdPost(alertId, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (AlertsApi.prototype.createAlertAlertsPost = function (
            alertId,
            category,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.AlertsApiFp)(this.configuration)
              .createAlertAlertsPost(alertId, category, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (AlertsApi.prototype.getAlertAlertsAlertIdGet = function (
            alertId,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.AlertsApiFp)(this.configuration)
              .getAlertAlertsAlertIdGet(alertId, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (AlertsApi.prototype.getAlertsAlertsGet = function (authorization, options) {
            var _this = this;
            return (0, exports.AlertsApiFp)(this.configuration)
              .getAlertsAlertsGet(authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          AlertsApi
        );
      })(base_1.BaseAPI);
      exports.AlertsApi = AlertsApi;
      exports.BeaconsApiAxiosParamCreator = function (configuration) {
        var _this = this;
        return {
          getBeaconBeaconsBeaconIdGet: function (beaconId, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'getBeaconBeaconsBeaconIdGet',
                      'beaconId',
                      beaconId,
                    ),
                    (localVarPath = '/beacons/{beacon_id}'.replace(
                      '{beacon_id}',
                      encodeURIComponent(String(beaconId)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getBeaconsBeaconsGet: function (authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    '/beacons',
                    (localVarUrlObj = new URL('/beacons', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          saveBeaconStateBeaconsPost: function (
            beaconId,
            online,
            category,
            activated,
            level,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'saveBeaconStateBeaconsPost',
                      'beaconId',
                      beaconId,
                    ),
                    (0, common_1.assertParamExists)('saveBeaconStateBeaconsPost', 'online', online),
                    (0, common_1.assertParamExists)(
                      'saveBeaconStateBeaconsPost',
                      'category',
                      category,
                    ),
                    (0, common_1.assertParamExists)(
                      'saveBeaconStateBeaconsPost',
                      'activated',
                      activated,
                    ),
                    (0, common_1.assertParamExists)('saveBeaconStateBeaconsPost', 'level', level),
                    '/beacons',
                    (localVarUrlObj = new URL('/beacons', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    void 0 !== beaconId && (localVarQueryParameter.beacon_id = beaconId),
                    void 0 !== online && (localVarQueryParameter.online = online),
                    void 0 !== category && (localVarQueryParameter.category = category),
                    void 0 !== activated && (localVarQueryParameter.activated = activated),
                    void 0 !== level && (localVarQueryParameter.level = level),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
        };
      };
      exports.BeaconsApiFp = function (configuration) {
        var localVarAxiosParamCreator = (0, exports.BeaconsApiAxiosParamCreator)(configuration);
        return {
          getBeaconBeaconsBeaconIdGet: function (beaconId, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getBeaconBeaconsBeaconIdGet(
                        beaconId,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'BeaconsApi.getBeaconBeaconsBeaconIdGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getBeaconsBeaconsGet: function (authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getBeaconsBeaconsGet(authorization, options),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b = base_1.operationServerMap['BeaconsApi.getBeaconsBeaconsGet']) ||
                            void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          saveBeaconStateBeaconsPost: function (
            beaconId,
            online,
            category,
            activated,
            level,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.saveBeaconStateBeaconsPost(
                        beaconId,
                        online,
                        category,
                        activated,
                        level,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'BeaconsApi.saveBeaconStateBeaconsPost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
        };
      };
      exports.BeaconsApiFactory = function (configuration, basePath, axios) {
        var localVarFp = (0, exports.BeaconsApiFp)(configuration);
        return {
          getBeaconBeaconsBeaconIdGet: function (beaconId, authorization, options) {
            return localVarFp
              .getBeaconBeaconsBeaconIdGet(beaconId, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getBeaconsBeaconsGet: function (authorization, options) {
            return localVarFp.getBeaconsBeaconsGet(authorization, options).then(function (request) {
              return request(axios, basePath);
            });
          },
          saveBeaconStateBeaconsPost: function (
            beaconId,
            online,
            category,
            activated,
            level,
            authorization,
            options,
          ) {
            return localVarFp
              .saveBeaconStateBeaconsPost(
                beaconId,
                online,
                category,
                activated,
                level,
                authorization,
                options,
              )
              .then(function (request) {
                return request(axios, basePath);
              });
          },
        };
      };
      var BeaconsApi = (function (_super) {
        function BeaconsApi() {
          return (null !== _super && _super.apply(this, arguments)) || this;
        }
        return (
          __extends(BeaconsApi, _super),
          (BeaconsApi.prototype.getBeaconBeaconsBeaconIdGet = function (
            beaconId,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.BeaconsApiFp)(this.configuration)
              .getBeaconBeaconsBeaconIdGet(beaconId, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (BeaconsApi.prototype.getBeaconsBeaconsGet = function (authorization, options) {
            var _this = this;
            return (0, exports.BeaconsApiFp)(this.configuration)
              .getBeaconsBeaconsGet(authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (BeaconsApi.prototype.saveBeaconStateBeaconsPost = function (
            beaconId,
            online,
            category,
            activated,
            level,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.BeaconsApiFp)(this.configuration)
              .saveBeaconStateBeaconsPost(
                beaconId,
                online,
                category,
                activated,
                level,
                authorization,
                options,
              )
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          BeaconsApi
        );
      })(base_1.BaseAPI);
      exports.BeaconsApi = BeaconsApi;
      exports.BuildingApiAxiosParamCreator = function (configuration) {
        var _this = this;
        return {
          getBuildingMapBuildingMapGet: function (authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    '/building_map',
                    (localVarUrlObj = new URL('/building_map', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
        };
      };
      exports.BuildingApiFp = function (configuration) {
        var localVarAxiosParamCreator = (0, exports.BuildingApiAxiosParamCreator)(configuration);
        return {
          getBuildingMapBuildingMapGet: function (authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getBuildingMapBuildingMapGet(
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'BuildingApi.getBuildingMapBuildingMapGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
        };
      };
      exports.BuildingApiFactory = function (configuration, basePath, axios) {
        var localVarFp = (0, exports.BuildingApiFp)(configuration);
        return {
          getBuildingMapBuildingMapGet: function (authorization, options) {
            return localVarFp
              .getBuildingMapBuildingMapGet(authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
        };
      };
      var BuildingApi = (function (_super) {
        function BuildingApi() {
          return (null !== _super && _super.apply(this, arguments)) || this;
        }
        return (
          __extends(BuildingApi, _super),
          (BuildingApi.prototype.getBuildingMapBuildingMapGet = function (authorization, options) {
            var _this = this;
            return (0, exports.BuildingApiFp)(this.configuration)
              .getBuildingMapBuildingMapGet(authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          BuildingApi
        );
      })(base_1.BaseAPI);
      exports.BuildingApi = BuildingApi;
      exports.DefaultApiAxiosParamCreator = function (configuration) {
        var _this = this;
        return {
          getEffectivePermissionsPermissionsGet: function (authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    '/permissions',
                    (localVarUrlObj = new URL('/permissions', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getTimeTimeGet: function (options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    '/time',
                    (localVarUrlObj = new URL('/time', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getUserUserGet: function (authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    '/user',
                    (localVarUrlObj = new URL('/user', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          lambdaSocketIoGet: function (options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    '/socket.io',
                    (localVarUrlObj = new URL('/socket.io', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
        };
      };
      exports.DefaultApiFp = function (configuration) {
        var localVarAxiosParamCreator = (0, exports.DefaultApiAxiosParamCreator)(configuration);
        return {
          getEffectivePermissionsPermissionsGet: function (authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getEffectivePermissionsPermissionsGet(
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'DefaultApi.getEffectivePermissionsPermissionsGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getTimeTimeGet: function (options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [4, localVarAxiosParamCreator.getTimeTimeGet(options)];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b = base_1.operationServerMap['DefaultApi.getTimeTimeGet']) ||
                            void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getUserUserGet: function (authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [4, localVarAxiosParamCreator.getUserUserGet(authorization, options)];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b = base_1.operationServerMap['DefaultApi.getUserUserGet']) ||
                            void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          lambdaSocketIoGet: function (options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [4, localVarAxiosParamCreator.lambdaSocketIoGet(options)];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b = base_1.operationServerMap['DefaultApi.lambdaSocketIoGet']) ||
                            void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
        };
      };
      exports.DefaultApiFactory = function (configuration, basePath, axios) {
        var localVarFp = (0, exports.DefaultApiFp)(configuration);
        return {
          getEffectivePermissionsPermissionsGet: function (authorization, options) {
            return localVarFp
              .getEffectivePermissionsPermissionsGet(authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getTimeTimeGet: function (options) {
            return localVarFp.getTimeTimeGet(options).then(function (request) {
              return request(axios, basePath);
            });
          },
          getUserUserGet: function (authorization, options) {
            return localVarFp.getUserUserGet(authorization, options).then(function (request) {
              return request(axios, basePath);
            });
          },
          lambdaSocketIoGet: function (options) {
            return localVarFp.lambdaSocketIoGet(options).then(function (request) {
              return request(axios, basePath);
            });
          },
        };
      };
      var DefaultApi = (function (_super) {
        function DefaultApi() {
          return (null !== _super && _super.apply(this, arguments)) || this;
        }
        return (
          __extends(DefaultApi, _super),
          (DefaultApi.prototype.getEffectivePermissionsPermissionsGet = function (
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.DefaultApiFp)(this.configuration)
              .getEffectivePermissionsPermissionsGet(authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (DefaultApi.prototype.getTimeTimeGet = function (options) {
            var _this = this;
            return (0, exports.DefaultApiFp)(this.configuration)
              .getTimeTimeGet(options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (DefaultApi.prototype.getUserUserGet = function (authorization, options) {
            var _this = this;
            return (0, exports.DefaultApiFp)(this.configuration)
              .getUserUserGet(authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (DefaultApi.prototype.lambdaSocketIoGet = function (options) {
            var _this = this;
            return (0, exports.DefaultApiFp)(this.configuration)
              .lambdaSocketIoGet(options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          DefaultApi
        );
      })(base_1.BaseAPI);
      exports.DefaultApi = DefaultApi;
      exports.DispensersApiAxiosParamCreator = function (configuration) {
        var _this = this;
        return {
          getDispenserHealthDispensersGuidHealthGet: function (guid, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'getDispenserHealthDispensersGuidHealthGet',
                      'guid',
                      guid,
                    ),
                    (localVarPath = '/dispensers/{guid}/health'.replace(
                      '{guid}',
                      encodeURIComponent(String(guid)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getDispenserStateDispensersGuidStateGet: function (guid, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'getDispenserStateDispensersGuidStateGet',
                      'guid',
                      guid,
                    ),
                    (localVarPath = '/dispensers/{guid}/state'.replace(
                      '{guid}',
                      encodeURIComponent(String(guid)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getDispensersDispensersGet: function (authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    '/dispensers',
                    (localVarUrlObj = new URL('/dispensers', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
        };
      };
      exports.DispensersApiFp = function (configuration) {
        var localVarAxiosParamCreator = (0, exports.DispensersApiAxiosParamCreator)(configuration);
        return {
          getDispenserHealthDispensersGuidHealthGet: function (guid, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getDispenserHealthDispensersGuidHealthGet(
                        guid,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'DispensersApi.getDispenserHealthDispensersGuidHealthGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getDispenserStateDispensersGuidStateGet: function (guid, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getDispenserStateDispensersGuidStateGet(
                        guid,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'DispensersApi.getDispenserStateDispensersGuidStateGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getDispensersDispensersGet: function (authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getDispensersDispensersGet(authorization, options),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'DispensersApi.getDispensersDispensersGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
        };
      };
      exports.DispensersApiFactory = function (configuration, basePath, axios) {
        var localVarFp = (0, exports.DispensersApiFp)(configuration);
        return {
          getDispenserHealthDispensersGuidHealthGet: function (guid, authorization, options) {
            return localVarFp
              .getDispenserHealthDispensersGuidHealthGet(guid, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getDispenserStateDispensersGuidStateGet: function (guid, authorization, options) {
            return localVarFp
              .getDispenserStateDispensersGuidStateGet(guid, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getDispensersDispensersGet: function (authorization, options) {
            return localVarFp
              .getDispensersDispensersGet(authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
        };
      };
      var DispensersApi = (function (_super) {
        function DispensersApi() {
          return (null !== _super && _super.apply(this, arguments)) || this;
        }
        return (
          __extends(DispensersApi, _super),
          (DispensersApi.prototype.getDispenserHealthDispensersGuidHealthGet = function (
            guid,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.DispensersApiFp)(this.configuration)
              .getDispenserHealthDispensersGuidHealthGet(guid, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (DispensersApi.prototype.getDispenserStateDispensersGuidStateGet = function (
            guid,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.DispensersApiFp)(this.configuration)
              .getDispenserStateDispensersGuidStateGet(guid, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (DispensersApi.prototype.getDispensersDispensersGet = function (authorization, options) {
            var _this = this;
            return (0, exports.DispensersApiFp)(this.configuration)
              .getDispensersDispensersGet(authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          DispensersApi
        );
      })(base_1.BaseAPI);
      exports.DispensersApi = DispensersApi;
      exports.DoorsApiAxiosParamCreator = function (configuration) {
        var _this = this;
        return {
          getDoorHealthDoorsDoorNameHealthGet: function (doorName, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'getDoorHealthDoorsDoorNameHealthGet',
                      'doorName',
                      doorName,
                    ),
                    (localVarPath = '/doors/{door_name}/health'.replace(
                      '{door_name}',
                      encodeURIComponent(String(doorName)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getDoorStateDoorsDoorNameStateGet: function (doorName, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'getDoorStateDoorsDoorNameStateGet',
                      'doorName',
                      doorName,
                    ),
                    (localVarPath = '/doors/{door_name}/state'.replace(
                      '{door_name}',
                      encodeURIComponent(String(doorName)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getDoorsDoorsGet: function (authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    '/doors',
                    (localVarUrlObj = new URL('/doors', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          postDoorRequestDoorsDoorNameRequestPost: function (
            doorName,
            doorRequest,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'postDoorRequestDoorsDoorNameRequestPost',
                      'doorName',
                      doorName,
                    ),
                    (0, common_1.assertParamExists)(
                      'postDoorRequestDoorsDoorNameRequestPost',
                      'doorRequest',
                      doorRequest,
                    ),
                    (localVarPath = '/doors/{door_name}/request'.replace(
                      '{door_name}',
                      encodeURIComponent(String(doorName)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      doorRequest,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
        };
      };
      exports.DoorsApiFp = function (configuration) {
        var localVarAxiosParamCreator = (0, exports.DoorsApiAxiosParamCreator)(configuration);
        return {
          getDoorHealthDoorsDoorNameHealthGet: function (doorName, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getDoorHealthDoorsDoorNameHealthGet(
                        doorName,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'DoorsApi.getDoorHealthDoorsDoorNameHealthGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getDoorStateDoorsDoorNameStateGet: function (doorName, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getDoorStateDoorsDoorNameStateGet(
                        doorName,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'DoorsApi.getDoorStateDoorsDoorNameStateGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getDoorsDoorsGet: function (authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [4, localVarAxiosParamCreator.getDoorsDoorsGet(authorization, options)];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b = base_1.operationServerMap['DoorsApi.getDoorsDoorsGet']) ||
                            void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          postDoorRequestDoorsDoorNameRequestPost: function (
            doorName,
            doorRequest,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.postDoorRequestDoorsDoorNameRequestPost(
                        doorName,
                        doorRequest,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'DoorsApi.postDoorRequestDoorsDoorNameRequestPost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
        };
      };
      exports.DoorsApiFactory = function (configuration, basePath, axios) {
        var localVarFp = (0, exports.DoorsApiFp)(configuration);
        return {
          getDoorHealthDoorsDoorNameHealthGet: function (doorName, authorization, options) {
            return localVarFp
              .getDoorHealthDoorsDoorNameHealthGet(doorName, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getDoorStateDoorsDoorNameStateGet: function (doorName, authorization, options) {
            return localVarFp
              .getDoorStateDoorsDoorNameStateGet(doorName, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getDoorsDoorsGet: function (authorization, options) {
            return localVarFp.getDoorsDoorsGet(authorization, options).then(function (request) {
              return request(axios, basePath);
            });
          },
          postDoorRequestDoorsDoorNameRequestPost: function (
            doorName,
            doorRequest,
            authorization,
            options,
          ) {
            return localVarFp
              .postDoorRequestDoorsDoorNameRequestPost(
                doorName,
                doorRequest,
                authorization,
                options,
              )
              .then(function (request) {
                return request(axios, basePath);
              });
          },
        };
      };
      var DoorsApi = (function (_super) {
        function DoorsApi() {
          return (null !== _super && _super.apply(this, arguments)) || this;
        }
        return (
          __extends(DoorsApi, _super),
          (DoorsApi.prototype.getDoorHealthDoorsDoorNameHealthGet = function (
            doorName,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.DoorsApiFp)(this.configuration)
              .getDoorHealthDoorsDoorNameHealthGet(doorName, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (DoorsApi.prototype.getDoorStateDoorsDoorNameStateGet = function (
            doorName,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.DoorsApiFp)(this.configuration)
              .getDoorStateDoorsDoorNameStateGet(doorName, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (DoorsApi.prototype.getDoorsDoorsGet = function (authorization, options) {
            var _this = this;
            return (0, exports.DoorsApiFp)(this.configuration)
              .getDoorsDoorsGet(authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (DoorsApi.prototype.postDoorRequestDoorsDoorNameRequestPost = function (
            doorName,
            doorRequest,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.DoorsApiFp)(this.configuration)
              .postDoorRequestDoorsDoorNameRequestPost(
                doorName,
                doorRequest,
                authorization,
                options,
              )
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          DoorsApi
        );
      })(base_1.BaseAPI);
      exports.DoorsApi = DoorsApi;
      exports.FleetsApiAxiosParamCreator = function (configuration) {
        var _this = this;
        return {
          getFleetLogFleetsNameLogGet: function (name, between, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)('getFleetLogFleetsNameLogGet', 'name', name),
                    (localVarPath = '/fleets/{name}/log'.replace(
                      '{name}',
                      encodeURIComponent(String(name)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    void 0 !== between && (localVarQueryParameter.between = between),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getFleetStateFleetsNameStateGet: function (name, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'getFleetStateFleetsNameStateGet',
                      'name',
                      name,
                    ),
                    (localVarPath = '/fleets/{name}/state'.replace(
                      '{name}',
                      encodeURIComponent(String(name)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getFleetsFleetsGet: function (authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    '/fleets',
                    (localVarUrlObj = new URL('/fleets', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
        };
      };
      exports.FleetsApiFp = function (configuration) {
        var localVarAxiosParamCreator = (0, exports.FleetsApiAxiosParamCreator)(configuration);
        return {
          getFleetLogFleetsNameLogGet: function (name, between, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getFleetLogFleetsNameLogGet(
                        name,
                        between,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'FleetsApi.getFleetLogFleetsNameLogGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getFleetStateFleetsNameStateGet: function (name, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getFleetStateFleetsNameStateGet(
                        name,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'FleetsApi.getFleetStateFleetsNameStateGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getFleetsFleetsGet: function (authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getFleetsFleetsGet(authorization, options),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b = base_1.operationServerMap['FleetsApi.getFleetsFleetsGet']) ||
                            void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
        };
      };
      exports.FleetsApiFactory = function (configuration, basePath, axios) {
        var localVarFp = (0, exports.FleetsApiFp)(configuration);
        return {
          getFleetLogFleetsNameLogGet: function (name, between, authorization, options) {
            return localVarFp
              .getFleetLogFleetsNameLogGet(name, between, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getFleetStateFleetsNameStateGet: function (name, authorization, options) {
            return localVarFp
              .getFleetStateFleetsNameStateGet(name, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getFleetsFleetsGet: function (authorization, options) {
            return localVarFp.getFleetsFleetsGet(authorization, options).then(function (request) {
              return request(axios, basePath);
            });
          },
        };
      };
      var FleetsApi = (function (_super) {
        function FleetsApi() {
          return (null !== _super && _super.apply(this, arguments)) || this;
        }
        return (
          __extends(FleetsApi, _super),
          (FleetsApi.prototype.getFleetLogFleetsNameLogGet = function (
            name,
            between,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.FleetsApiFp)(this.configuration)
              .getFleetLogFleetsNameLogGet(name, between, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (FleetsApi.prototype.getFleetStateFleetsNameStateGet = function (
            name,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.FleetsApiFp)(this.configuration)
              .getFleetStateFleetsNameStateGet(name, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (FleetsApi.prototype.getFleetsFleetsGet = function (authorization, options) {
            var _this = this;
            return (0, exports.FleetsApiFp)(this.configuration)
              .getFleetsFleetsGet(authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          FleetsApi
        );
      })(base_1.BaseAPI);
      exports.FleetsApi = FleetsApi;
      exports.IngestorsApiAxiosParamCreator = function (configuration) {
        var _this = this;
        return {
          getIngestorHealthIngestorsGuidHealthGet: function (guid, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'getIngestorHealthIngestorsGuidHealthGet',
                      'guid',
                      guid,
                    ),
                    (localVarPath = '/ingestors/{guid}/health'.replace(
                      '{guid}',
                      encodeURIComponent(String(guid)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getIngestorStateIngestorsGuidStateGet: function (guid, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'getIngestorStateIngestorsGuidStateGet',
                      'guid',
                      guid,
                    ),
                    (localVarPath = '/ingestors/{guid}/state'.replace(
                      '{guid}',
                      encodeURIComponent(String(guid)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getIngestorsIngestorsGet: function (authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    '/ingestors',
                    (localVarUrlObj = new URL('/ingestors', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
        };
      };
      exports.IngestorsApiFp = function (configuration) {
        var localVarAxiosParamCreator = (0, exports.IngestorsApiAxiosParamCreator)(configuration);
        return {
          getIngestorHealthIngestorsGuidHealthGet: function (guid, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getIngestorHealthIngestorsGuidHealthGet(
                        guid,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'IngestorsApi.getIngestorHealthIngestorsGuidHealthGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getIngestorStateIngestorsGuidStateGet: function (guid, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getIngestorStateIngestorsGuidStateGet(
                        guid,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'IngestorsApi.getIngestorStateIngestorsGuidStateGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getIngestorsIngestorsGet: function (authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getIngestorsIngestorsGet(authorization, options),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'IngestorsApi.getIngestorsIngestorsGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
        };
      };
      exports.IngestorsApiFactory = function (configuration, basePath, axios) {
        var localVarFp = (0, exports.IngestorsApiFp)(configuration);
        return {
          getIngestorHealthIngestorsGuidHealthGet: function (guid, authorization, options) {
            return localVarFp
              .getIngestorHealthIngestorsGuidHealthGet(guid, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getIngestorStateIngestorsGuidStateGet: function (guid, authorization, options) {
            return localVarFp
              .getIngestorStateIngestorsGuidStateGet(guid, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getIngestorsIngestorsGet: function (authorization, options) {
            return localVarFp
              .getIngestorsIngestorsGet(authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
        };
      };
      var IngestorsApi = (function (_super) {
        function IngestorsApi() {
          return (null !== _super && _super.apply(this, arguments)) || this;
        }
        return (
          __extends(IngestorsApi, _super),
          (IngestorsApi.prototype.getIngestorHealthIngestorsGuidHealthGet = function (
            guid,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.IngestorsApiFp)(this.configuration)
              .getIngestorHealthIngestorsGuidHealthGet(guid, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (IngestorsApi.prototype.getIngestorStateIngestorsGuidStateGet = function (
            guid,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.IngestorsApiFp)(this.configuration)
              .getIngestorStateIngestorsGuidStateGet(guid, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (IngestorsApi.prototype.getIngestorsIngestorsGet = function (authorization, options) {
            var _this = this;
            return (0, exports.IngestorsApiFp)(this.configuration)
              .getIngestorsIngestorsGet(authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          IngestorsApi
        );
      })(base_1.BaseAPI);
      exports.IngestorsApi = IngestorsApi;
      exports.LiftsApiAxiosParamCreator = function (configuration) {
        var _this = this;
        return {
          getLiftHealthLiftsLiftNameHealthGet: function (liftName, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'getLiftHealthLiftsLiftNameHealthGet',
                      'liftName',
                      liftName,
                    ),
                    (localVarPath = '/lifts/{lift_name}/health'.replace(
                      '{lift_name}',
                      encodeURIComponent(String(liftName)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getLiftStateLiftsLiftNameStateGet: function (liftName, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'getLiftStateLiftsLiftNameStateGet',
                      'liftName',
                      liftName,
                    ),
                    (localVarPath = '/lifts/{lift_name}/state'.replace(
                      '{lift_name}',
                      encodeURIComponent(String(liftName)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getLiftsLiftsGet: function (authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    '/lifts',
                    (localVarUrlObj = new URL('/lifts', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          postLiftRequestLiftsLiftNameRequestPost: function (
            liftName,
            liftRequest,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'postLiftRequestLiftsLiftNameRequestPost',
                      'liftName',
                      liftName,
                    ),
                    (0, common_1.assertParamExists)(
                      'postLiftRequestLiftsLiftNameRequestPost',
                      'liftRequest',
                      liftRequest,
                    ),
                    (localVarPath = '/lifts/{lift_name}/request'.replace(
                      '{lift_name}',
                      encodeURIComponent(String(liftName)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      liftRequest,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
        };
      };
      exports.LiftsApiFp = function (configuration) {
        var localVarAxiosParamCreator = (0, exports.LiftsApiAxiosParamCreator)(configuration);
        return {
          getLiftHealthLiftsLiftNameHealthGet: function (liftName, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getLiftHealthLiftsLiftNameHealthGet(
                        liftName,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'LiftsApi.getLiftHealthLiftsLiftNameHealthGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getLiftStateLiftsLiftNameStateGet: function (liftName, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getLiftStateLiftsLiftNameStateGet(
                        liftName,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'LiftsApi.getLiftStateLiftsLiftNameStateGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getLiftsLiftsGet: function (authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [4, localVarAxiosParamCreator.getLiftsLiftsGet(authorization, options)];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b = base_1.operationServerMap['LiftsApi.getLiftsLiftsGet']) ||
                            void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          postLiftRequestLiftsLiftNameRequestPost: function (
            liftName,
            liftRequest,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.postLiftRequestLiftsLiftNameRequestPost(
                        liftName,
                        liftRequest,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'LiftsApi.postLiftRequestLiftsLiftNameRequestPost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
        };
      };
      exports.LiftsApiFactory = function (configuration, basePath, axios) {
        var localVarFp = (0, exports.LiftsApiFp)(configuration);
        return {
          getLiftHealthLiftsLiftNameHealthGet: function (liftName, authorization, options) {
            return localVarFp
              .getLiftHealthLiftsLiftNameHealthGet(liftName, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getLiftStateLiftsLiftNameStateGet: function (liftName, authorization, options) {
            return localVarFp
              .getLiftStateLiftsLiftNameStateGet(liftName, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getLiftsLiftsGet: function (authorization, options) {
            return localVarFp.getLiftsLiftsGet(authorization, options).then(function (request) {
              return request(axios, basePath);
            });
          },
          postLiftRequestLiftsLiftNameRequestPost: function (
            liftName,
            liftRequest,
            authorization,
            options,
          ) {
            return localVarFp
              .postLiftRequestLiftsLiftNameRequestPost(
                liftName,
                liftRequest,
                authorization,
                options,
              )
              .then(function (request) {
                return request(axios, basePath);
              });
          },
        };
      };
      var LiftsApi = (function (_super) {
        function LiftsApi() {
          return (null !== _super && _super.apply(this, arguments)) || this;
        }
        return (
          __extends(LiftsApi, _super),
          (LiftsApi.prototype.getLiftHealthLiftsLiftNameHealthGet = function (
            liftName,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.LiftsApiFp)(this.configuration)
              .getLiftHealthLiftsLiftNameHealthGet(liftName, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (LiftsApi.prototype.getLiftStateLiftsLiftNameStateGet = function (
            liftName,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.LiftsApiFp)(this.configuration)
              .getLiftStateLiftsLiftNameStateGet(liftName, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (LiftsApi.prototype.getLiftsLiftsGet = function (authorization, options) {
            var _this = this;
            return (0, exports.LiftsApiFp)(this.configuration)
              .getLiftsLiftsGet(authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (LiftsApi.prototype.postLiftRequestLiftsLiftNameRequestPost = function (
            liftName,
            liftRequest,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.LiftsApiFp)(this.configuration)
              .postLiftRequestLiftsLiftNameRequestPost(
                liftName,
                liftRequest,
                authorization,
                options,
              )
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          LiftsApi
        );
      })(base_1.BaseAPI);
      exports.LiftsApi = LiftsApi;
      exports.TasksApiAxiosParamCreator = function (configuration) {
        var _this = this;
        return {
          delScheduledTasksEventScheduledTasksTaskIdClearPut: function (
            taskId,
            eventDate,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'delScheduledTasksEventScheduledTasksTaskIdClearPut',
                      'taskId',
                      taskId,
                    ),
                    (0, common_1.assertParamExists)(
                      'delScheduledTasksEventScheduledTasksTaskIdClearPut',
                      'eventDate',
                      eventDate,
                    ),
                    (localVarPath = '/scheduled_tasks/{task_id}/clear'.replace(
                      '{task_id}',
                      encodeURIComponent(String(taskId)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'PUT' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    void 0 !== eventDate &&
                      (localVarQueryParameter.event_date =
                        eventDate instanceof Date ? eventDate.toISOString() : eventDate),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          delScheduledTasksScheduledTasksTaskIdDelete: function (taskId, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'delScheduledTasksScheduledTasksTaskIdDelete',
                      'taskId',
                      taskId,
                    ),
                    (localVarPath = '/scheduled_tasks/{task_id}'.replace(
                      '{task_id}',
                      encodeURIComponent(String(taskId)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'DELETE' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          deleteFavoriteTaskFavoriteTasksFavoriteTaskIdDelete: function (
            favoriteTaskId,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'deleteFavoriteTaskFavoriteTasksFavoriteTaskIdDelete',
                      'favoriteTaskId',
                      favoriteTaskId,
                    ),
                    (localVarPath = '/favorite_tasks/{favorite_task_id}'.replace(
                      '{favorite_task_id}',
                      encodeURIComponent(String(favoriteTaskId)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'DELETE' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getFavoritesTasksFavoriteTasksGet: function (authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    '/favorite_tasks',
                    (localVarUrlObj = new URL('/favorite_tasks', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getScheduledTaskScheduledTasksTaskIdGet: function (taskId, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'getScheduledTaskScheduledTasksTaskIdGet',
                      'taskId',
                      taskId,
                    ),
                    (localVarPath = '/scheduled_tasks/{task_id}'.replace(
                      '{task_id}',
                      encodeURIComponent(String(taskId)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getScheduledTasksScheduledTasksGet: function (
            startBefore,
            untilAfter,
            limit,
            offset,
            orderBy,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'getScheduledTasksScheduledTasksGet',
                      'startBefore',
                      startBefore,
                    ),
                    (0, common_1.assertParamExists)(
                      'getScheduledTasksScheduledTasksGet',
                      'untilAfter',
                      untilAfter,
                    ),
                    '/scheduled_tasks',
                    (localVarUrlObj = new URL('/scheduled_tasks', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    void 0 !== startBefore &&
                      (localVarQueryParameter.start_before =
                        startBefore instanceof Date ? startBefore.toISOString() : startBefore),
                    void 0 !== untilAfter &&
                      (localVarQueryParameter.until_after =
                        untilAfter instanceof Date ? untilAfter.toISOString() : untilAfter),
                    void 0 !== limit && (localVarQueryParameter.limit = limit),
                    void 0 !== offset && (localVarQueryParameter.offset = offset),
                    void 0 !== orderBy && (localVarQueryParameter.order_by = orderBy),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getTaskLogTasksTaskIdLogGet: function (taskId, between, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'getTaskLogTasksTaskIdLogGet',
                      'taskId',
                      taskId,
                    ),
                    (localVarPath = '/tasks/{task_id}/log'.replace(
                      '{task_id}',
                      encodeURIComponent(String(taskId)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    void 0 !== between && (localVarQueryParameter.between = between),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getTaskRequestTasksTaskIdRequestGet: function (taskId, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'getTaskRequestTasksTaskIdRequestGet',
                      'taskId',
                      taskId,
                    ),
                    (localVarPath = '/tasks/{task_id}/request'.replace(
                      '{task_id}',
                      encodeURIComponent(String(taskId)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          getTaskStateTasksTaskIdStateGet: function (taskId, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'getTaskStateTasksTaskIdStateGet',
                      'taskId',
                      taskId,
                    ),
                    (localVarPath = '/tasks/{task_id}/state'.replace(
                      '{task_id}',
                      encodeURIComponent(String(taskId)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          postActivityDiscoveryTasksActivityDiscoveryPost: function (
            activityDiscoveryRequest,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'postActivityDiscoveryTasksActivityDiscoveryPost',
                      'activityDiscoveryRequest',
                      activityDiscoveryRequest,
                    ),
                    '/tasks/activity_discovery',
                    (localVarUrlObj = new URL(
                      '/tasks/activity_discovery',
                      common_1.DUMMY_BASE_URL,
                    )),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      activityDiscoveryRequest,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          postCancelTaskTasksCancelTaskPost: function (cancelTaskRequest, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'postCancelTaskTasksCancelTaskPost',
                      'cancelTaskRequest',
                      cancelTaskRequest,
                    ),
                    '/tasks/cancel_task',
                    (localVarUrlObj = new URL('/tasks/cancel_task', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      cancelTaskRequest,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          postDispatchTaskTasksDispatchTaskPost: function (
            dispatchTaskRequest,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'postDispatchTaskTasksDispatchTaskPost',
                      'dispatchTaskRequest',
                      dispatchTaskRequest,
                    ),
                    '/tasks/dispatch_task',
                    (localVarUrlObj = new URL('/tasks/dispatch_task', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      dispatchTaskRequest,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          postFavoriteTaskFavoriteTasksPost: function (
            taskFavoritePydantic,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'postFavoriteTaskFavoriteTasksPost',
                      'taskFavoritePydantic',
                      taskFavoritePydantic,
                    ),
                    '/favorite_tasks',
                    (localVarUrlObj = new URL('/favorite_tasks', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      taskFavoritePydantic,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          postInterruptTaskTasksInterruptTaskPost: function (
            taskInterruptionRequest,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'postInterruptTaskTasksInterruptTaskPost',
                      'taskInterruptionRequest',
                      taskInterruptionRequest,
                    ),
                    '/tasks/interrupt_task',
                    (localVarUrlObj = new URL('/tasks/interrupt_task', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      taskInterruptionRequest,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          postKillTaskTasksKillTaskPost: function (taskKillRequest, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'postKillTaskTasksKillTaskPost',
                      'taskKillRequest',
                      taskKillRequest,
                    ),
                    '/tasks/kill_task',
                    (localVarUrlObj = new URL('/tasks/kill_task', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      taskKillRequest,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          postResumeTaskTasksResumeTaskPost: function (taskResumeRequest, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'postResumeTaskTasksResumeTaskPost',
                      'taskResumeRequest',
                      taskResumeRequest,
                    ),
                    '/tasks/resume_task',
                    (localVarUrlObj = new URL('/tasks/resume_task', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      taskResumeRequest,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          postRewindTaskTasksRewindTaskPost: function (taskRewindRequest, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'postRewindTaskTasksRewindTaskPost',
                      'taskRewindRequest',
                      taskRewindRequest,
                    ),
                    '/tasks/rewind_task',
                    (localVarUrlObj = new URL('/tasks/rewind_task', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      taskRewindRequest,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          postRobotTaskTasksRobotTaskPost: function (robotTaskRequest, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'postRobotTaskTasksRobotTaskPost',
                      'robotTaskRequest',
                      robotTaskRequest,
                    ),
                    '/tasks/robot_task',
                    (localVarUrlObj = new URL('/tasks/robot_task', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      robotTaskRequest,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          postScheduledTaskScheduledTasksPost: function (
            postScheduledTaskRequest,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'postScheduledTaskScheduledTasksPost',
                      'postScheduledTaskRequest',
                      postScheduledTaskRequest,
                    ),
                    '/scheduled_tasks',
                    (localVarUrlObj = new URL('/scheduled_tasks', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      postScheduledTaskRequest,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          postSkipPhaseTasksSkipPhasePost: function (taskPhaseSkipRequest, authorization, options) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'postSkipPhaseTasksSkipPhasePost',
                      'taskPhaseSkipRequest',
                      taskPhaseSkipRequest,
                    ),
                    '/tasks/skip_phase',
                    (localVarUrlObj = new URL('/tasks/skip_phase', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      taskPhaseSkipRequest,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          postTaskDiscoveryTasksTaskDiscoveryPost: function (
            taskDiscoveryRequest,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'postTaskDiscoveryTasksTaskDiscoveryPost',
                      'taskDiscoveryRequest',
                      taskDiscoveryRequest,
                    ),
                    '/tasks/task_discovery',
                    (localVarUrlObj = new URL('/tasks/task_discovery', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      taskDiscoveryRequest,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          postUndoSkipPhaseTasksUndoSkipPhasePost: function (
            undoPhaseSkipRequest,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'postUndoSkipPhaseTasksUndoSkipPhasePost',
                      'undoPhaseSkipRequest',
                      undoPhaseSkipRequest,
                    ),
                    '/tasks/undo_skip_phase',
                    (localVarUrlObj = new URL('/tasks/undo_skip_phase', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      undoPhaseSkipRequest,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          queryTaskStatesTasksGet: function (
            taskId,
            category,
            assignedTo,
            status,
            startTimeBetween,
            finishTimeBetween,
            limit,
            offset,
            orderBy,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    '/tasks',
                    (localVarUrlObj = new URL('/tasks', common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'GET' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    void 0 !== taskId && (localVarQueryParameter.task_id = taskId),
                    void 0 !== category && (localVarQueryParameter.category = category),
                    void 0 !== assignedTo && (localVarQueryParameter.assigned_to = assignedTo),
                    void 0 !== status && (localVarQueryParameter.status = status),
                    void 0 !== startTimeBetween &&
                      (localVarQueryParameter.start_time_between = startTimeBetween),
                    void 0 !== finishTimeBetween &&
                      (localVarQueryParameter.finish_time_between = finishTimeBetween),
                    void 0 !== limit && (localVarQueryParameter.limit = limit),
                    void 0 !== offset && (localVarQueryParameter.offset = offset),
                    void 0 !== orderBy && (localVarQueryParameter.order_by = orderBy),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
          updateScheduleTaskScheduledTasksTaskIdUpdatePost: function (
            taskId,
            postScheduledTaskRequest,
            exceptDate,
            authorization,
            options,
          ) {
            return (
              void 0 === options && (options = {}),
              __awaiter(_this, void 0, void 0, function () {
                var localVarPath,
                  localVarUrlObj,
                  baseOptions,
                  localVarRequestOptions,
                  localVarHeaderParameter,
                  localVarQueryParameter,
                  headersFromBaseOptions;
                return __generator(this, function (_a) {
                  return (
                    (0, common_1.assertParamExists)(
                      'updateScheduleTaskScheduledTasksTaskIdUpdatePost',
                      'taskId',
                      taskId,
                    ),
                    (0, common_1.assertParamExists)(
                      'updateScheduleTaskScheduledTasksTaskIdUpdatePost',
                      'postScheduledTaskRequest',
                      postScheduledTaskRequest,
                    ),
                    (localVarPath = '/scheduled_tasks/{task_id}/update'.replace(
                      '{task_id}',
                      encodeURIComponent(String(taskId)),
                    )),
                    (localVarUrlObj = new URL(localVarPath, common_1.DUMMY_BASE_URL)),
                    configuration && (baseOptions = configuration.baseOptions),
                    (localVarRequestOptions = __assign(
                      __assign({ method: 'POST' }, baseOptions),
                      options,
                    )),
                    (localVarHeaderParameter = {}),
                    (localVarQueryParameter = {}),
                    void 0 !== exceptDate &&
                      (localVarQueryParameter.except_date =
                        exceptDate instanceof Date ? exceptDate.toISOString() : exceptDate),
                    null != authorization &&
                      (localVarHeaderParameter.authorization = String(authorization)),
                    (localVarHeaderParameter['Content-Type'] = 'application/json'),
                    (0, common_1.setSearchParams)(localVarUrlObj, localVarQueryParameter),
                    (headersFromBaseOptions =
                      baseOptions && baseOptions.headers ? baseOptions.headers : {}),
                    (localVarRequestOptions.headers = __assign(
                      __assign(__assign({}, localVarHeaderParameter), headersFromBaseOptions),
                      options.headers,
                    )),
                    (localVarRequestOptions.data = (0, common_1.serializeDataIfNeeded)(
                      postScheduledTaskRequest,
                      localVarRequestOptions,
                      configuration,
                    )),
                    [
                      2,
                      {
                        url: (0, common_1.toPathString)(localVarUrlObj),
                        options: localVarRequestOptions,
                      },
                    ]
                  );
                });
              })
            );
          },
        };
      };
      exports.TasksApiFp = function (configuration) {
        var localVarAxiosParamCreator = (0, exports.TasksApiAxiosParamCreator)(configuration);
        return {
          delScheduledTasksEventScheduledTasksTaskIdClearPut: function (
            taskId,
            eventDate,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.delScheduledTasksEventScheduledTasksTaskIdClearPut(
                        taskId,
                        eventDate,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.delScheduledTasksEventScheduledTasksTaskIdClearPut'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          delScheduledTasksScheduledTasksTaskIdDelete: function (taskId, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.delScheduledTasksScheduledTasksTaskIdDelete(
                        taskId,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.delScheduledTasksScheduledTasksTaskIdDelete'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          deleteFavoriteTaskFavoriteTasksFavoriteTaskIdDelete: function (
            favoriteTaskId,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.deleteFavoriteTaskFavoriteTasksFavoriteTaskIdDelete(
                        favoriteTaskId,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.deleteFavoriteTaskFavoriteTasksFavoriteTaskIdDelete'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getFavoritesTasksFavoriteTasksGet: function (authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getFavoritesTasksFavoriteTasksGet(
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.getFavoritesTasksFavoriteTasksGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getScheduledTaskScheduledTasksTaskIdGet: function (taskId, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getScheduledTaskScheduledTasksTaskIdGet(
                        taskId,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.getScheduledTaskScheduledTasksTaskIdGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getScheduledTasksScheduledTasksGet: function (
            startBefore,
            untilAfter,
            limit,
            offset,
            orderBy,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getScheduledTasksScheduledTasksGet(
                        startBefore,
                        untilAfter,
                        limit,
                        offset,
                        orderBy,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.getScheduledTasksScheduledTasksGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getTaskLogTasksTaskIdLogGet: function (taskId, between, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getTaskLogTasksTaskIdLogGet(
                        taskId,
                        between,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.getTaskLogTasksTaskIdLogGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getTaskRequestTasksTaskIdRequestGet: function (taskId, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getTaskRequestTasksTaskIdRequestGet(
                        taskId,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.getTaskRequestTasksTaskIdRequestGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          getTaskStateTasksTaskIdStateGet: function (taskId, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.getTaskStateTasksTaskIdStateGet(
                        taskId,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.getTaskStateTasksTaskIdStateGet'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          postActivityDiscoveryTasksActivityDiscoveryPost: function (
            activityDiscoveryRequest,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.postActivityDiscoveryTasksActivityDiscoveryPost(
                        activityDiscoveryRequest,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.postActivityDiscoveryTasksActivityDiscoveryPost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          postCancelTaskTasksCancelTaskPost: function (cancelTaskRequest, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.postCancelTaskTasksCancelTaskPost(
                        cancelTaskRequest,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.postCancelTaskTasksCancelTaskPost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          postDispatchTaskTasksDispatchTaskPost: function (
            dispatchTaskRequest,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.postDispatchTaskTasksDispatchTaskPost(
                        dispatchTaskRequest,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.postDispatchTaskTasksDispatchTaskPost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          postFavoriteTaskFavoriteTasksPost: function (
            taskFavoritePydantic,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.postFavoriteTaskFavoriteTasksPost(
                        taskFavoritePydantic,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.postFavoriteTaskFavoriteTasksPost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          postInterruptTaskTasksInterruptTaskPost: function (
            taskInterruptionRequest,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.postInterruptTaskTasksInterruptTaskPost(
                        taskInterruptionRequest,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.postInterruptTaskTasksInterruptTaskPost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          postKillTaskTasksKillTaskPost: function (taskKillRequest, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.postKillTaskTasksKillTaskPost(
                        taskKillRequest,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.postKillTaskTasksKillTaskPost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          postResumeTaskTasksResumeTaskPost: function (taskResumeRequest, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.postResumeTaskTasksResumeTaskPost(
                        taskResumeRequest,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.postResumeTaskTasksResumeTaskPost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          postRewindTaskTasksRewindTaskPost: function (taskRewindRequest, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.postRewindTaskTasksRewindTaskPost(
                        taskRewindRequest,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.postRewindTaskTasksRewindTaskPost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          postRobotTaskTasksRobotTaskPost: function (robotTaskRequest, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.postRobotTaskTasksRobotTaskPost(
                        robotTaskRequest,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.postRobotTaskTasksRobotTaskPost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          postScheduledTaskScheduledTasksPost: function (
            postScheduledTaskRequest,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.postScheduledTaskScheduledTasksPost(
                        postScheduledTaskRequest,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.postScheduledTaskScheduledTasksPost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          postSkipPhaseTasksSkipPhasePost: function (taskPhaseSkipRequest, authorization, options) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.postSkipPhaseTasksSkipPhasePost(
                        taskPhaseSkipRequest,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.postSkipPhaseTasksSkipPhasePost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          postTaskDiscoveryTasksTaskDiscoveryPost: function (
            taskDiscoveryRequest,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.postTaskDiscoveryTasksTaskDiscoveryPost(
                        taskDiscoveryRequest,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.postTaskDiscoveryTasksTaskDiscoveryPost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          postUndoSkipPhaseTasksUndoSkipPhasePost: function (
            undoPhaseSkipRequest,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.postUndoSkipPhaseTasksUndoSkipPhasePost(
                        undoPhaseSkipRequest,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.postUndoSkipPhaseTasksUndoSkipPhasePost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          queryTaskStatesTasksGet: function (
            taskId,
            category,
            assignedTo,
            status,
            startTimeBetween,
            finishTimeBetween,
            limit,
            offset,
            orderBy,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.queryTaskStatesTasksGet(
                        taskId,
                        category,
                        assignedTo,
                        status,
                        startTimeBetween,
                        finishTimeBetween,
                        limit,
                        offset,
                        orderBy,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap['TasksApi.queryTaskStatesTasksGet']) ||
                            void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
          updateScheduleTaskScheduledTasksTaskIdUpdatePost: function (
            taskId,
            postScheduledTaskRequest,
            exceptDate,
            authorization,
            options,
          ) {
            var _a, _b, _c;
            return __awaiter(this, void 0, void 0, function () {
              var localVarAxiosArgs, localVarOperationServerIndex, localVarOperationServerBasePath;
              return __generator(this, function (_d) {
                switch (_d.label) {
                  case 0:
                    return [
                      4,
                      localVarAxiosParamCreator.updateScheduleTaskScheduledTasksTaskIdUpdatePost(
                        taskId,
                        postScheduledTaskRequest,
                        exceptDate,
                        authorization,
                        options,
                      ),
                    ];
                  case 1:
                    return (
                      (localVarAxiosArgs = _d.sent()),
                      (localVarOperationServerIndex =
                        null !==
                          (_a = null == configuration ? void 0 : configuration.serverIndex) &&
                        void 0 !== _a
                          ? _a
                          : 0),
                      (localVarOperationServerBasePath =
                        null ===
                          (_c =
                            null ===
                              (_b =
                                base_1.operationServerMap[
                                  'TasksApi.updateScheduleTaskScheduledTasksTaskIdUpdatePost'
                                ]) || void 0 === _b
                              ? void 0
                              : _b[localVarOperationServerIndex]) || void 0 === _c
                          ? void 0
                          : _c.url),
                      [
                        2,
                        function (axios, basePath) {
                          return (0, common_1.createRequestFunction)(
                            localVarAxiosArgs,
                            axios_1.default,
                            base_1.BASE_PATH,
                            configuration,
                          )(axios, localVarOperationServerBasePath || basePath);
                        },
                      ]
                    );
                }
              });
            });
          },
        };
      };
      exports.TasksApiFactory = function (configuration, basePath, axios) {
        var localVarFp = (0, exports.TasksApiFp)(configuration);
        return {
          delScheduledTasksEventScheduledTasksTaskIdClearPut: function (
            taskId,
            eventDate,
            authorization,
            options,
          ) {
            return localVarFp
              .delScheduledTasksEventScheduledTasksTaskIdClearPut(
                taskId,
                eventDate,
                authorization,
                options,
              )
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          delScheduledTasksScheduledTasksTaskIdDelete: function (taskId, authorization, options) {
            return localVarFp
              .delScheduledTasksScheduledTasksTaskIdDelete(taskId, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          deleteFavoriteTaskFavoriteTasksFavoriteTaskIdDelete: function (
            favoriteTaskId,
            authorization,
            options,
          ) {
            return localVarFp
              .deleteFavoriteTaskFavoriteTasksFavoriteTaskIdDelete(
                favoriteTaskId,
                authorization,
                options,
              )
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getFavoritesTasksFavoriteTasksGet: function (authorization, options) {
            return localVarFp
              .getFavoritesTasksFavoriteTasksGet(authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getScheduledTaskScheduledTasksTaskIdGet: function (taskId, authorization, options) {
            return localVarFp
              .getScheduledTaskScheduledTasksTaskIdGet(taskId, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getScheduledTasksScheduledTasksGet: function (
            startBefore,
            untilAfter,
            limit,
            offset,
            orderBy,
            authorization,
            options,
          ) {
            return localVarFp
              .getScheduledTasksScheduledTasksGet(
                startBefore,
                untilAfter,
                limit,
                offset,
                orderBy,
                authorization,
                options,
              )
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getTaskLogTasksTaskIdLogGet: function (taskId, between, authorization, options) {
            return localVarFp
              .getTaskLogTasksTaskIdLogGet(taskId, between, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getTaskRequestTasksTaskIdRequestGet: function (taskId, authorization, options) {
            return localVarFp
              .getTaskRequestTasksTaskIdRequestGet(taskId, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          getTaskStateTasksTaskIdStateGet: function (taskId, authorization, options) {
            return localVarFp
              .getTaskStateTasksTaskIdStateGet(taskId, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          postActivityDiscoveryTasksActivityDiscoveryPost: function (
            activityDiscoveryRequest,
            authorization,
            options,
          ) {
            return localVarFp
              .postActivityDiscoveryTasksActivityDiscoveryPost(
                activityDiscoveryRequest,
                authorization,
                options,
              )
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          postCancelTaskTasksCancelTaskPost: function (cancelTaskRequest, authorization, options) {
            return localVarFp
              .postCancelTaskTasksCancelTaskPost(cancelTaskRequest, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          postDispatchTaskTasksDispatchTaskPost: function (
            dispatchTaskRequest,
            authorization,
            options,
          ) {
            return localVarFp
              .postDispatchTaskTasksDispatchTaskPost(dispatchTaskRequest, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          postFavoriteTaskFavoriteTasksPost: function (
            taskFavoritePydantic,
            authorization,
            options,
          ) {
            return localVarFp
              .postFavoriteTaskFavoriteTasksPost(taskFavoritePydantic, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          postInterruptTaskTasksInterruptTaskPost: function (
            taskInterruptionRequest,
            authorization,
            options,
          ) {
            return localVarFp
              .postInterruptTaskTasksInterruptTaskPost(
                taskInterruptionRequest,
                authorization,
                options,
              )
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          postKillTaskTasksKillTaskPost: function (taskKillRequest, authorization, options) {
            return localVarFp
              .postKillTaskTasksKillTaskPost(taskKillRequest, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          postResumeTaskTasksResumeTaskPost: function (taskResumeRequest, authorization, options) {
            return localVarFp
              .postResumeTaskTasksResumeTaskPost(taskResumeRequest, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          postRewindTaskTasksRewindTaskPost: function (taskRewindRequest, authorization, options) {
            return localVarFp
              .postRewindTaskTasksRewindTaskPost(taskRewindRequest, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          postRobotTaskTasksRobotTaskPost: function (robotTaskRequest, authorization, options) {
            return localVarFp
              .postRobotTaskTasksRobotTaskPost(robotTaskRequest, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          postScheduledTaskScheduledTasksPost: function (
            postScheduledTaskRequest,
            authorization,
            options,
          ) {
            return localVarFp
              .postScheduledTaskScheduledTasksPost(postScheduledTaskRequest, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          postSkipPhaseTasksSkipPhasePost: function (taskPhaseSkipRequest, authorization, options) {
            return localVarFp
              .postSkipPhaseTasksSkipPhasePost(taskPhaseSkipRequest, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          postTaskDiscoveryTasksTaskDiscoveryPost: function (
            taskDiscoveryRequest,
            authorization,
            options,
          ) {
            return localVarFp
              .postTaskDiscoveryTasksTaskDiscoveryPost(taskDiscoveryRequest, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          postUndoSkipPhaseTasksUndoSkipPhasePost: function (
            undoPhaseSkipRequest,
            authorization,
            options,
          ) {
            return localVarFp
              .postUndoSkipPhaseTasksUndoSkipPhasePost(undoPhaseSkipRequest, authorization, options)
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          queryTaskStatesTasksGet: function (
            taskId,
            category,
            assignedTo,
            status,
            startTimeBetween,
            finishTimeBetween,
            limit,
            offset,
            orderBy,
            authorization,
            options,
          ) {
            return localVarFp
              .queryTaskStatesTasksGet(
                taskId,
                category,
                assignedTo,
                status,
                startTimeBetween,
                finishTimeBetween,
                limit,
                offset,
                orderBy,
                authorization,
                options,
              )
              .then(function (request) {
                return request(axios, basePath);
              });
          },
          updateScheduleTaskScheduledTasksTaskIdUpdatePost: function (
            taskId,
            postScheduledTaskRequest,
            exceptDate,
            authorization,
            options,
          ) {
            return localVarFp
              .updateScheduleTaskScheduledTasksTaskIdUpdatePost(
                taskId,
                postScheduledTaskRequest,
                exceptDate,
                authorization,
                options,
              )
              .then(function (request) {
                return request(axios, basePath);
              });
          },
        };
      };
      var TasksApi = (function (_super) {
        function TasksApi() {
          return (null !== _super && _super.apply(this, arguments)) || this;
        }
        return (
          __extends(TasksApi, _super),
          (TasksApi.prototype.delScheduledTasksEventScheduledTasksTaskIdClearPut = function (
            taskId,
            eventDate,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .delScheduledTasksEventScheduledTasksTaskIdClearPut(
                taskId,
                eventDate,
                authorization,
                options,
              )
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.delScheduledTasksScheduledTasksTaskIdDelete = function (
            taskId,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .delScheduledTasksScheduledTasksTaskIdDelete(taskId, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.deleteFavoriteTaskFavoriteTasksFavoriteTaskIdDelete = function (
            favoriteTaskId,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .deleteFavoriteTaskFavoriteTasksFavoriteTaskIdDelete(
                favoriteTaskId,
                authorization,
                options,
              )
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.getFavoritesTasksFavoriteTasksGet = function (
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .getFavoritesTasksFavoriteTasksGet(authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.getScheduledTaskScheduledTasksTaskIdGet = function (
            taskId,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .getScheduledTaskScheduledTasksTaskIdGet(taskId, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.getScheduledTasksScheduledTasksGet = function (
            startBefore,
            untilAfter,
            limit,
            offset,
            orderBy,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .getScheduledTasksScheduledTasksGet(
                startBefore,
                untilAfter,
                limit,
                offset,
                orderBy,
                authorization,
                options,
              )
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.getTaskLogTasksTaskIdLogGet = function (
            taskId,
            between,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .getTaskLogTasksTaskIdLogGet(taskId, between, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.getTaskRequestTasksTaskIdRequestGet = function (
            taskId,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .getTaskRequestTasksTaskIdRequestGet(taskId, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.getTaskStateTasksTaskIdStateGet = function (
            taskId,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .getTaskStateTasksTaskIdStateGet(taskId, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.postActivityDiscoveryTasksActivityDiscoveryPost = function (
            activityDiscoveryRequest,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .postActivityDiscoveryTasksActivityDiscoveryPost(
                activityDiscoveryRequest,
                authorization,
                options,
              )
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.postCancelTaskTasksCancelTaskPost = function (
            cancelTaskRequest,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .postCancelTaskTasksCancelTaskPost(cancelTaskRequest, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.postDispatchTaskTasksDispatchTaskPost = function (
            dispatchTaskRequest,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .postDispatchTaskTasksDispatchTaskPost(dispatchTaskRequest, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.postFavoriteTaskFavoriteTasksPost = function (
            taskFavoritePydantic,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .postFavoriteTaskFavoriteTasksPost(taskFavoritePydantic, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.postInterruptTaskTasksInterruptTaskPost = function (
            taskInterruptionRequest,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .postInterruptTaskTasksInterruptTaskPost(
                taskInterruptionRequest,
                authorization,
                options,
              )
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.postKillTaskTasksKillTaskPost = function (
            taskKillRequest,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .postKillTaskTasksKillTaskPost(taskKillRequest, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.postResumeTaskTasksResumeTaskPost = function (
            taskResumeRequest,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .postResumeTaskTasksResumeTaskPost(taskResumeRequest, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.postRewindTaskTasksRewindTaskPost = function (
            taskRewindRequest,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .postRewindTaskTasksRewindTaskPost(taskRewindRequest, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.postRobotTaskTasksRobotTaskPost = function (
            robotTaskRequest,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .postRobotTaskTasksRobotTaskPost(robotTaskRequest, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.postScheduledTaskScheduledTasksPost = function (
            postScheduledTaskRequest,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .postScheduledTaskScheduledTasksPost(postScheduledTaskRequest, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.postSkipPhaseTasksSkipPhasePost = function (
            taskPhaseSkipRequest,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .postSkipPhaseTasksSkipPhasePost(taskPhaseSkipRequest, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.postTaskDiscoveryTasksTaskDiscoveryPost = function (
            taskDiscoveryRequest,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .postTaskDiscoveryTasksTaskDiscoveryPost(taskDiscoveryRequest, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.postUndoSkipPhaseTasksUndoSkipPhasePost = function (
            undoPhaseSkipRequest,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .postUndoSkipPhaseTasksUndoSkipPhasePost(undoPhaseSkipRequest, authorization, options)
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.queryTaskStatesTasksGet = function (
            taskId,
            category,
            assignedTo,
            status,
            startTimeBetween,
            finishTimeBetween,
            limit,
            offset,
            orderBy,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .queryTaskStatesTasksGet(
                taskId,
                category,
                assignedTo,
                status,
                startTimeBetween,
                finishTimeBetween,
                limit,
                offset,
                orderBy,
                authorization,
                options,
              )
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          (TasksApi.prototype.updateScheduleTaskScheduledTasksTaskIdUpdatePost = function (
            taskId,
            postScheduledTaskRequest,
            exceptDate,
            authorization,
            options,
          ) {
            var _this = this;
            return (0, exports.TasksApiFp)(this.configuration)
              .updateScheduleTaskScheduledTasksTaskIdUpdatePost(
                taskId,
                postScheduledTaskRequest,
                exceptDate,
                authorization,
                options,
              )
              .then(function (request) {
                return request(_this.axios, _this.basePath);
              });
          }),
          TasksApi
        );
      })(base_1.BaseAPI);
      exports.TasksApi = TasksApi;
    },
    '../api-client/dist/lib/openapi/base.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var extendStatics,
        __extends =
          (this && this.__extends) ||
          ((extendStatics = function (d, b) {
            return (
              (extendStatics =
                Object.setPrototypeOf ||
                ({ __proto__: [] } instanceof Array &&
                  function (d, b) {
                    d.__proto__ = b;
                  }) ||
                function (d, b) {
                  for (var p in b) Object.prototype.hasOwnProperty.call(b, p) && (d[p] = b[p]);
                }),
              extendStatics(d, b)
            );
          }),
          function (d, b) {
            if ('function' != typeof b && null !== b)
              throw new TypeError(
                'Class extends value ' + String(b) + ' is not a constructor or null',
              );
            function __() {
              this.constructor = d;
            }
            extendStatics(d, b),
              (d.prototype =
                null === b ? Object.create(b) : ((__.prototype = b.prototype), new __()));
          }),
        __importDefault =
          (this && this.__importDefault) ||
          function (mod) {
            return mod && mod.__esModule ? mod : { default: mod };
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.operationServerMap =
          exports.RequiredError =
          exports.BaseAPI =
          exports.COLLECTION_FORMATS =
          exports.BASE_PATH =
            void 0);
      var axios_1 = __importDefault(
        __webpack_require__(
          '../../node_modules/.pnpm/axios@1.6.8_debug@4.3.4/node_modules/axios/dist/browser/axios.cjs',
        ),
      );
      (exports.BASE_PATH = 'http://localhost'.replace(/\/+$/, '')),
        (exports.COLLECTION_FORMATS = { csv: ',', ssv: ' ', tsv: '\t', pipes: '|' });
      var BaseAPI = function BaseAPI(configuration, basePath, axios) {
        var _a;
        void 0 === basePath && (basePath = exports.BASE_PATH),
          void 0 === axios && (axios = axios_1.default),
          (this.basePath = basePath),
          (this.axios = axios),
          configuration &&
            ((this.configuration = configuration),
            (this.basePath =
              null !== (_a = configuration.basePath) && void 0 !== _a ? _a : basePath));
      };
      exports.BaseAPI = BaseAPI;
      var RequiredError = (function (_super) {
        function RequiredError(field, msg) {
          var _this = _super.call(this, msg) || this;
          return (_this.field = field), (_this.name = 'RequiredError'), _this;
        }
        return __extends(RequiredError, _super), RequiredError;
      })(Error);
      (exports.RequiredError = RequiredError), (exports.operationServerMap = {});
    },
    '../api-client/dist/lib/openapi/common.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __assign =
          (this && this.__assign) ||
          function () {
            return (
              (__assign =
                Object.assign ||
                function (t) {
                  for (var s, i = 1, n = arguments.length; i < n; i++)
                    for (var p in (s = arguments[i]))
                      Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                  return t;
                }),
              __assign.apply(this, arguments)
            );
          },
        __awaiter =
          (this && this.__awaiter) ||
          function (thisArg, _arguments, P, generator) {
            return new (P || (P = Promise))(function (resolve, reject) {
              function fulfilled(value) {
                try {
                  step(generator.next(value));
                } catch (e) {
                  reject(e);
                }
              }
              function rejected(value) {
                try {
                  step(generator.throw(value));
                } catch (e) {
                  reject(e);
                }
              }
              function step(result) {
                result.done
                  ? resolve(result.value)
                  : (function adopt(value) {
                      return value instanceof P
                        ? value
                        : new P(function (resolve) {
                            resolve(value);
                          });
                    })(result.value).then(fulfilled, rejected);
              }
              step((generator = generator.apply(thisArg, _arguments || [])).next());
            });
          },
        __generator =
          (this && this.__generator) ||
          function (thisArg, body) {
            var f,
              y,
              t,
              g,
              _ = {
                label: 0,
                sent: function () {
                  if (1 & t[0]) throw t[1];
                  return t[1];
                },
                trys: [],
                ops: [],
              };
            return (
              (g = { next: verb(0), throw: verb(1), return: verb(2) }),
              'function' == typeof Symbol &&
                (g[Symbol.iterator] = function () {
                  return this;
                }),
              g
            );
            function verb(n) {
              return function (v) {
                return (function step(op) {
                  if (f) throw new TypeError('Generator is already executing.');
                  for (; _; )
                    try {
                      if (
                        ((f = 1),
                        y &&
                          (t =
                            2 & op[0]
                              ? y.return
                              : op[0]
                                ? y.throw || ((t = y.return) && t.call(y), 0)
                                : y.next) &&
                          !(t = t.call(y, op[1])).done)
                      )
                        return t;
                      switch (((y = 0), t && (op = [2 & op[0], t.value]), op[0])) {
                        case 0:
                        case 1:
                          t = op;
                          break;
                        case 4:
                          return _.label++, { value: op[1], done: !1 };
                        case 5:
                          _.label++, (y = op[1]), (op = [0]);
                          continue;
                        case 7:
                          (op = _.ops.pop()), _.trys.pop();
                          continue;
                        default:
                          if (
                            !((t = _.trys),
                            (t = t.length > 0 && t[t.length - 1]) || (6 !== op[0] && 2 !== op[0]))
                          ) {
                            _ = 0;
                            continue;
                          }
                          if (3 === op[0] && (!t || (op[1] > t[0] && op[1] < t[3]))) {
                            _.label = op[1];
                            break;
                          }
                          if (6 === op[0] && _.label < t[1]) {
                            (_.label = t[1]), (t = op);
                            break;
                          }
                          if (t && _.label < t[2]) {
                            (_.label = t[2]), _.ops.push(op);
                            break;
                          }
                          t[2] && _.ops.pop(), _.trys.pop();
                          continue;
                      }
                      op = body.call(thisArg, _);
                    } catch (e) {
                      (op = [6, e]), (y = 0);
                    } finally {
                      f = t = 0;
                    }
                  if (5 & op[0]) throw op[1];
                  return { value: op[0] ? op[1] : void 0, done: !0 };
                })([n, v]);
              };
            }
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.createRequestFunction =
          exports.toPathString =
          exports.serializeDataIfNeeded =
          exports.setSearchParams =
          exports.setOAuthToObject =
          exports.setBearerAuthToObject =
          exports.setBasicAuthToObject =
          exports.setApiKeyToObject =
          exports.assertParamExists =
          exports.DUMMY_BASE_URL =
            void 0);
      var base_1 = __webpack_require__('../api-client/dist/lib/openapi/base.js');
      exports.DUMMY_BASE_URL = 'https://example.com';
      exports.assertParamExists = function (functionName, paramName, paramValue) {
        if (null == paramValue)
          throw new base_1.RequiredError(
            paramName,
            'Required parameter ' +
              paramName +
              ' was null or undefined when calling ' +
              functionName +
              '.',
          );
      };
      exports.setApiKeyToObject = function (object, keyParamName, configuration) {
        return __awaiter(this, void 0, void 0, function () {
          var localVarApiKeyValue, _a;
          return __generator(this, function (_b) {
            switch (_b.label) {
              case 0:
                return configuration && configuration.apiKey
                  ? 'function' != typeof configuration.apiKey
                    ? [3, 2]
                    : [4, configuration.apiKey(keyParamName)]
                  : [3, 5];
              case 1:
                return (_a = _b.sent()), [3, 4];
              case 2:
                return [4, configuration.apiKey];
              case 3:
                (_a = _b.sent()), (_b.label = 4);
              case 4:
                (localVarApiKeyValue = _a),
                  (object[keyParamName] = localVarApiKeyValue),
                  (_b.label = 5);
              case 5:
                return [2];
            }
          });
        });
      };
      exports.setBasicAuthToObject = function (object, configuration) {
        configuration &&
          (configuration.username || configuration.password) &&
          (object.auth = { username: configuration.username, password: configuration.password });
      };
      exports.setBearerAuthToObject = function (object, configuration) {
        return __awaiter(this, void 0, void 0, function () {
          var accessToken, _a;
          return __generator(this, function (_b) {
            switch (_b.label) {
              case 0:
                return configuration && configuration.accessToken
                  ? 'function' != typeof configuration.accessToken
                    ? [3, 2]
                    : [4, configuration.accessToken()]
                  : [3, 5];
              case 1:
                return (_a = _b.sent()), [3, 4];
              case 2:
                return [4, configuration.accessToken];
              case 3:
                (_a = _b.sent()), (_b.label = 4);
              case 4:
                (accessToken = _a),
                  (object.Authorization = 'Bearer ' + accessToken),
                  (_b.label = 5);
              case 5:
                return [2];
            }
          });
        });
      };
      function setFlattenedQueryParams(urlSearchParams, parameter, key) {
        void 0 === key && (key = ''),
          null != parameter &&
            ('object' == typeof parameter
              ? Array.isArray(parameter)
                ? parameter.forEach(function (item) {
                    return setFlattenedQueryParams(urlSearchParams, item, key);
                  })
                : Object.keys(parameter).forEach(function (currentKey) {
                    return setFlattenedQueryParams(
                      urlSearchParams,
                      parameter[currentKey],
                      key + ('' !== key ? '.' : '') + currentKey,
                    );
                  })
              : urlSearchParams.has(key)
                ? urlSearchParams.append(key, parameter)
                : urlSearchParams.set(key, parameter));
      }
      exports.setOAuthToObject = function (object, name, scopes, configuration) {
        return __awaiter(this, void 0, void 0, function () {
          var localVarAccessTokenValue, _a;
          return __generator(this, function (_b) {
            switch (_b.label) {
              case 0:
                return configuration && configuration.accessToken
                  ? 'function' != typeof configuration.accessToken
                    ? [3, 2]
                    : [4, configuration.accessToken(name, scopes)]
                  : [3, 5];
              case 1:
                return (_a = _b.sent()), [3, 4];
              case 2:
                return [4, configuration.accessToken];
              case 3:
                (_a = _b.sent()), (_b.label = 4);
              case 4:
                (localVarAccessTokenValue = _a),
                  (object.Authorization = 'Bearer ' + localVarAccessTokenValue),
                  (_b.label = 5);
              case 5:
                return [2];
            }
          });
        });
      };
      exports.setSearchParams = function (url) {
        for (var objects = [], _i = 1; _i < arguments.length; _i++) objects[_i - 1] = arguments[_i];
        var searchParams = new URLSearchParams(url.search);
        setFlattenedQueryParams(searchParams, objects), (url.search = searchParams.toString());
      };
      exports.serializeDataIfNeeded = function (value, requestOptions, configuration) {
        var nonString = 'string' != typeof value;
        return (
          nonString && configuration && configuration.isJsonMime
            ? configuration.isJsonMime(requestOptions.headers['Content-Type'])
            : nonString
        )
          ? JSON.stringify(void 0 !== value ? value : {})
          : value || '';
      };
      exports.toPathString = function (url) {
        return url.pathname + url.search + url.hash;
      };
      exports.createRequestFunction = function (axiosArgs, globalAxios, BASE_PATH, configuration) {
        return function (axios, basePath) {
          var _a;
          void 0 === axios && (axios = globalAxios), void 0 === basePath && (basePath = BASE_PATH);
          var axiosRequestArgs = __assign(__assign({}, axiosArgs.options), {
            url:
              (axios.defaults.baseURL
                ? ''
                : null !== (_a = null == configuration ? void 0 : configuration.basePath) &&
                    void 0 !== _a
                  ? _a
                  : basePath) + axiosArgs.url,
          });
          return axios.request(axiosRequestArgs);
        };
      };
    },
    '../api-client/dist/lib/openapi/configuration.js': (__unused_webpack_module, exports) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.Configuration = void 0);
      var Configuration = (function () {
        function Configuration(param) {
          void 0 === param && (param = {}),
            (this.apiKey = param.apiKey),
            (this.username = param.username),
            (this.password = param.password),
            (this.accessToken = param.accessToken),
            (this.basePath = param.basePath),
            (this.serverIndex = param.serverIndex),
            (this.baseOptions = param.baseOptions),
            (this.formDataCtor = param.formDataCtor);
        }
        return (
          (Configuration.prototype.isJsonMime = function (mime) {
            var jsonMime = new RegExp(
              '^(application/json|[^;/ \t]+/[^;/ \t]+[+]json)[ \t]*(;.*)?$',
              'i',
            );
            return (
              null !== mime &&
              (jsonMime.test(mime) || 'application/json-patch+json' === mime.toLowerCase())
            );
          }),
          Configuration
        );
      })();
      exports.Configuration = Configuration;
    },
    '../api-client/dist/lib/openapi/index.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __createBinding =
          (this && this.__createBinding) ||
          (Object.create
            ? function (o, m, k, k2) {
                void 0 === k2 && (k2 = k),
                  Object.defineProperty(o, k2, {
                    enumerable: !0,
                    get: function () {
                      return m[k];
                    },
                  });
              }
            : function (o, m, k, k2) {
                void 0 === k2 && (k2 = k), (o[k2] = m[k]);
              }),
        __exportStar =
          (this && this.__exportStar) ||
          function (m, exports) {
            for (var p in m)
              'default' === p ||
                Object.prototype.hasOwnProperty.call(exports, p) ||
                __createBinding(exports, m, p);
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        __exportStar(__webpack_require__('../api-client/dist/lib/openapi/api.js'), exports),
        __exportStar(
          __webpack_require__('../api-client/dist/lib/openapi/configuration.js'),
          exports,
        );
    },
    './src/components/app-contexts.tsx': (
      __unused_webpack_module,
      __webpack_exports__,
      __webpack_require__,
    ) => {
      'use strict';
      __webpack_require__.d(__webpack_exports__, { J3: () => AppControllerContext });
      var extendStatics,
        react = __webpack_require__(
          '../../node_modules/.pnpm/react@18.2.0/node_modules/react/index.js',
        ),
        browser = __webpack_require__(
          '../../node_modules/.pnpm/debug@4.3.4_supports-color@5.5.0/node_modules/debug/src/browser.js',
        ),
        browser_default = __webpack_require__.n(browser),
        eventemitter3 = __webpack_require__(
          '../../node_modules/.pnpm/eventemitter3@4.0.7/node_modules/eventemitter3/index.js',
        ),
        eventemitter3_default = __webpack_require__.n(eventemitter3),
        keycloak = __webpack_require__(
          '../../node_modules/.pnpm/keycloak-js@11.0.3/node_modules/keycloak-js/dist/keycloak.js',
        ),
        keycloak_default = __webpack_require__.n(keycloak),
        __extends =
          ((extendStatics = function (d, b) {
            return (
              (extendStatics =
                Object.setPrototypeOf ||
                ({ __proto__: [] } instanceof Array &&
                  function (d, b) {
                    d.__proto__ = b;
                  }) ||
                function (d, b) {
                  for (var p in b) Object.prototype.hasOwnProperty.call(b, p) && (d[p] = b[p]);
                }),
              extendStatics(d, b)
            );
          }),
          function (d, b) {
            if ('function' != typeof b && null !== b)
              throw new TypeError(
                'Class extends value ' + String(b) + ' is not a constructor or null',
              );
            function __() {
              this.constructor = d;
            }
            extendStatics(d, b),
              (d.prototype =
                null === b ? Object.create(b) : ((__.prototype = b.prototype), new __()));
          }),
        __awaiter = function (thisArg, _arguments, P, generator) {
          return new (P || (P = Promise))(function (resolve, reject) {
            function fulfilled(value) {
              try {
                step(generator.next(value));
              } catch (e) {
                reject(e);
              }
            }
            function rejected(value) {
              try {
                step(generator.throw(value));
              } catch (e) {
                reject(e);
              }
            }
            function step(result) {
              result.done
                ? resolve(result.value)
                : (function adopt(value) {
                    return value instanceof P
                      ? value
                      : new P(function (resolve) {
                          resolve(value);
                        });
                  })(result.value).then(fulfilled, rejected);
            }
            step((generator = generator.apply(thisArg, _arguments || [])).next());
          });
        },
        __generator = function (thisArg, body) {
          var f,
            y,
            t,
            g,
            _ = {
              label: 0,
              sent: function () {
                if (1 & t[0]) throw t[1];
                return t[1];
              },
              trys: [],
              ops: [],
            };
          return (
            (g = { next: verb(0), throw: verb(1), return: verb(2) }),
            'function' == typeof Symbol &&
              (g[Symbol.iterator] = function () {
                return this;
              }),
            g
          );
          function verb(n) {
            return function (v) {
              return (function step(op) {
                if (f) throw new TypeError('Generator is already executing.');
                for (; _; )
                  try {
                    if (
                      ((f = 1),
                      y &&
                        (t =
                          2 & op[0]
                            ? y.return
                            : op[0]
                              ? y.throw || ((t = y.return) && t.call(y), 0)
                              : y.next) &&
                        !(t = t.call(y, op[1])).done)
                    )
                      return t;
                    switch (((y = 0), t && (op = [2 & op[0], t.value]), op[0])) {
                      case 0:
                      case 1:
                        t = op;
                        break;
                      case 4:
                        return _.label++, { value: op[1], done: !1 };
                      case 5:
                        _.label++, (y = op[1]), (op = [0]);
                        continue;
                      case 7:
                        (op = _.ops.pop()), _.trys.pop();
                        continue;
                      default:
                        if (
                          !((t = _.trys),
                          (t = t.length > 0 && t[t.length - 1]) || (6 !== op[0] && 2 !== op[0]))
                        ) {
                          _ = 0;
                          continue;
                        }
                        if (3 === op[0] && (!t || (op[1] > t[0] && op[1] < t[3]))) {
                          _.label = op[1];
                          break;
                        }
                        if (6 === op[0] && _.label < t[1]) {
                          (_.label = t[1]), (t = op);
                          break;
                        }
                        if (t && _.label < t[2]) {
                          (_.label = t[2]), _.ops.push(op);
                          break;
                        }
                        t[2] && _.ops.pop(), _.trys.pop();
                        continue;
                    }
                    op = body.call(thisArg, _);
                  } catch (e) {
                    (op = [6, e]), (y = 0);
                  } finally {
                    f = t = 0;
                  }
                if (5 & op[0]) throw op[1];
                return { value: op[0] ? op[1] : void 0, done: !0 };
              })([n, v]);
            };
          }
        },
        debug = browser_default()('authenticator'),
        KeycloakAuthenticator = (function (_super) {
          function KeycloakAuthenticator(config, silentCheckSsoRedirectUri) {
            var _this = _super.call(this) || this;
            return (
              (_this._initialized = !1),
              (_this._inst = keycloak_default()(config)),
              (_this._silentCheckSsoRedirectUri = silentCheckSsoRedirectUri),
              _this
            );
          }
          return (
            __extends(KeycloakAuthenticator, _super),
            Object.defineProperty(KeycloakAuthenticator.prototype, 'user', {
              get: function () {
                return this._user;
              },
              enumerable: !1,
              configurable: !0,
            }),
            Object.defineProperty(KeycloakAuthenticator.prototype, 'token', {
              get: function () {
                return this._inst.token;
              },
              enumerable: !1,
              configurable: !0,
            }),
            (KeycloakAuthenticator.prototype._getUser = function () {
              return this._inst.idTokenParsed.preferred_username;
            }),
            (KeycloakAuthenticator.prototype.init = function () {
              return __awaiter(this, void 0, void 0, function () {
                var _this = this;
                return __generator(this, function (_b) {
                  switch (_b.label) {
                    case 0:
                      return this._initialized
                        ? (debug('already initialized'), [2])
                        : (debug('initializing authenticator'),
                          (this._inst.onAuthSuccess = function () {
                            return __awaiter(_this, void 0, void 0, function () {
                              return __generator(this, function (_a) {
                                return (
                                  (this._user = this._getUser()),
                                  debug('authenticated as', this._user),
                                  this.emit('userChanged', this._user),
                                  [2]
                                );
                              });
                            });
                          }),
                          (this._inst.onAuthLogout = function () {
                            debug('logout'),
                              (_this._user = void 0),
                              _this.emit('userChanged', null);
                          }),
                          [
                            4,
                            this._inst.init({
                              onLoad: 'check-sso',
                              silentCheckSsoRedirectUri: this._silentCheckSsoRedirectUri,
                            }),
                          ]);
                    case 1:
                      _b.sent(), (_b.label = 2);
                    case 2:
                      return _b.trys.push([2, 4, , 5]), [4, this._inst.updateToken(30)];
                    case 3:
                      return _b.sent() && debug('token refreshed'), [3, 5];
                    case 4:
                      return _b.sent(), debug('token not refreshed'), [3, 5];
                    case 5:
                      return (
                        (this._user = this._inst.tokenParsed && this._getUser()),
                        (this._initialized = !0),
                        [2]
                      );
                  }
                });
              });
            }),
            (KeycloakAuthenticator.prototype.refreshToken = function () {
              return __awaiter(this, void 0, void 0, function () {
                return __generator(this, function (_a) {
                  switch (_a.label) {
                    case 0:
                      return this._initialized ? [4, this._inst.updateToken(5)] : [3, 2];
                    case 1:
                      _a.sent()
                        ? ((this._user = this._getUser()), this.emit('tokenRefresh', null))
                        : debug('token not refreshed'),
                        (_a.label = 2);
                    case 2:
                      return [2];
                  }
                });
              });
            }),
            (KeycloakAuthenticator.prototype.login = function (successRedirectUri) {
              return __awaiter(this, void 0, void 0, function () {
                return __generator(this, function (_a) {
                  switch (_a.label) {
                    case 0:
                      return [4, this._inst.login({ redirectUri: successRedirectUri })];
                    case 1:
                      throw (_a.sent(), new Error('should not reach here'));
                  }
                });
              });
            }),
            (KeycloakAuthenticator.prototype.logout = function () {
              return __awaiter(this, void 0, void 0, function () {
                return __generator(this, function (_a) {
                  switch (_a.label) {
                    case 0:
                      return [4, this._inst.logout()];
                    case 1:
                      throw (_a.sent(), new Error('should not reach here'));
                  }
                });
              });
            }),
            KeycloakAuthenticator
          );
        })(eventemitter3_default());
      var stub_extends = (function () {
          var extendStatics = function (d, b) {
            return (
              (extendStatics =
                Object.setPrototypeOf ||
                ({ __proto__: [] } instanceof Array &&
                  function (d, b) {
                    d.__proto__ = b;
                  }) ||
                function (d, b) {
                  for (var p in b) Object.prototype.hasOwnProperty.call(b, p) && (d[p] = b[p]);
                }),
              extendStatics(d, b)
            );
          };
          return function (d, b) {
            if ('function' != typeof b && null !== b)
              throw new TypeError(
                'Class extends value ' + String(b) + ' is not a constructor or null',
              );
            function __() {
              this.constructor = d;
            }
            extendStatics(d, b),
              (d.prototype =
                null === b ? Object.create(b) : ((__.prototype = b.prototype), new __()));
          };
        })(),
        StubAuthenticator = (function (_super) {
          function StubAuthenticator(user, token) {
            void 0 === user && (user = 'stub'), void 0 === token && (token = void 0);
            var _this = _super.call(this) || this;
            return (_this.user = user), (_this.token = token), _this;
          }
          return (
            stub_extends(StubAuthenticator, _super),
            (StubAuthenticator.prototype.init = function () {
              return Promise.resolve();
            }),
            (StubAuthenticator.prototype.login = function () {
              throw new Error('not supported');
            }),
            (StubAuthenticator.prototype.logout = function () {
              throw new Error('not supported');
            }),
            (StubAuthenticator.prototype.refreshToken = function () {
              return Promise.resolve();
            }),
            StubAuthenticator
          );
        })(eventemitter3_default());
      var jsx_runtime = __webpack_require__(
        '../../node_modules/.pnpm/react@18.2.0/node_modules/react/jsx-runtime.js',
      );
      __webpack_require__('../api-client/dist/lib/index.js');
      var styled = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/styles/styled.js',
        ),
        Typography = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Typography/Typography.js',
        ),
        Button = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Button/Button.js',
        ),
        login_card_assign = function () {
          return (
            (login_card_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            login_card_assign.apply(this, arguments)
          );
        },
        classes = {
          container: 'login-card-container',
          title: 'login-card-title',
          logo: 'login-card-logo',
        },
        StyledDiv = (0, styled.Ay)('div')(function (_a) {
          var _b,
            theme = _a.theme;
          return (
            ((_b = {})['&.' + classes.container] = {
              display: 'flex',
              flexDirection: 'column',
              alignItems: 'center',
              borderStyle: 'none',
              borderRadius: 20,
              borderColor: 'black',
              padding: '70px',
              width: 'fit-content',
              minWidth: 250,
              backgroundColor: 'snow',
              boxShadow: theme.shadows[12],
            }),
            (_b['& .' + classes.title] = { color: '#44497a' }),
            (_b['& .' + classes.logo] = { width: 100, margin: '25px 0px 50px 0px' }),
            _b
          );
        }),
        login_card_LoginCard = react.forwardRef(function (_a, ref) {
          var title = _a.title,
            logo = _a.logo,
            onLoginClick = _a.onLoginClick,
            children = _a.children;
          return (0, jsx_runtime.jsxs)(
            StyledDiv,
            login_card_assign(
              { ref, className: classes.container },
              {
                children: [
                  (0, jsx_runtime.jsx)(
                    Typography.A,
                    login_card_assign(
                      { variant: 'h4', className: classes.title },
                      { children: title },
                    ),
                    void 0,
                  ),
                  (0, jsx_runtime.jsx)(
                    'img',
                    { src: logo, alt: '', className: classes.logo },
                    void 0,
                  ),
                  (0, jsx_runtime.jsx)(
                    Button.A,
                    login_card_assign(
                      { variant: 'contained', 'aria-label': 'Login', onClick: onLoginClick },
                      { children: 'Login' },
                    ),
                    void 0,
                  ),
                  children,
                ],
              },
            ),
            void 0,
          );
        });
      login_card_LoginCard.__docgenInfo = {
        description: '',
        methods: [],
        displayName: 'LoginCard',
      };
      var login_page_classes = { container: 'login-page-container' };
      (0, styled.Ay)('div')(function (_a) {
        var _b,
          theme = _a.theme;
        return (
          ((_b = {})['&.' + login_page_classes.container] = {
            width: '100vw',
            height: '100vh',
            position: 'absolute',
            left: 0,
            top: 0,
            backgroundColor: theme.palette.primary.main,
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
          }),
          _b
        );
      });
      function _defineProperty(obj, key, value) {
        return (
          (key = (function _toPropertyKey(t) {
            var i = (function _toPrimitive(t, r) {
              if ('object' != typeof t || !t) return t;
              var e = t[Symbol.toPrimitive];
              if (void 0 !== e) {
                var i = e.call(t, r || 'default');
                if ('object' != typeof i) return i;
                throw new TypeError('@@toPrimitive must return a primitive value.');
              }
              return ('string' === r ? String : Number)(t);
            })(t, 'string');
            return 'symbol' == typeof i ? i : i + '';
          })(key)) in obj
            ? Object.defineProperty(obj, key, {
                value,
                enumerable: !0,
                configurable: !0,
                writable: !0,
              })
            : (obj[key] = value),
          obj
        );
      }
      const resource_manager_dispensers_debug = browser_default()('ResourceManager');
      class DispenserResourceManager {
        constructor(dispenserResources) {
          _defineProperty(this, 'getIconPath', async (dispenserName) => {
            if (!this.dispenserExists(dispenserName))
              return (
                resource_manager_dispensers_debug(
                  'failed to load icon for "'.concat(
                    dispenserName,
                    '" (dispenser does not exist in resources)',
                  ),
                ),
                null
              );
            if (!this.dispensers[dispenserName].hasOwnProperty('icons'))
              return (
                resource_manager_dispensers_debug(
                  'failed to load icon for "'.concat(
                    dispenserName,
                    '" (dispenser does not have an icon)',
                  ),
                ),
                null
              );
            const dispenserIcon = this.dispensers[dispenserName].icons[dispenserName];
            try {
              return (
                await __webpack_require__('./src/assets eager recursive ^\\.\\/resources.*$')(
                  './resources'.concat(dispenserIcon),
                )
              ).default;
            } catch {
              return (
                resource_manager_dispensers_debug(
                  'failed to load icon for "'.concat(
                    dispenserName,
                    '" (failed to load icon module)',
                  ),
                ),
                null
              );
            }
          }),
            _defineProperty(
              this,
              'dispenserExists',
              (dispenserName) => !!this.dispensers.hasOwnProperty(dispenserName),
            ),
            (this.dispensers = this.assignGuidToDispensers(dispenserResources));
        }
        get all() {
          return this.dispensers;
        }
        get allValues() {
          return Object.values(this.dispensers);
        }
        assignGuidToDispensers(dispensers) {
          const newDict = Object.assign({}, dispensers);
          return (
            Object.keys(dispensers).forEach((key) => {
              newDict[key].guid = key;
            }),
            newDict
          );
        }
      }
      let ThemeMode = (function (ThemeMode) {
        return (
          (ThemeMode[(ThemeMode.Default = 0)] = 'Default'),
          (ThemeMode[(ThemeMode.RmfLight = 1)] = 'RmfLight'),
          (ThemeMode[(ThemeMode.RmfDark = 2)] = 'RmfDark'),
          ThemeMode
        );
      })({});
      function defaultSettings() {
        return { themeMode: ThemeMode.Default };
      }
      function resource_manager_logos_defineProperty(obj, key, value) {
        return (
          (key = (function resource_manager_logos_toPropertyKey(t) {
            var i = (function resource_manager_logos_toPrimitive(t, r) {
              if ('object' != typeof t || !t) return t;
              var e = t[Symbol.toPrimitive];
              if (void 0 !== e) {
                var i = e.call(t, r || 'default');
                if ('object' != typeof i) return i;
                throw new TypeError('@@toPrimitive must return a primitive value.');
              }
              return ('string' === r ? String : Number)(t);
            })(t, 'string');
            return 'symbol' == typeof i ? i : i + '';
          })(key)) in obj
            ? Object.defineProperty(obj, key, {
                value,
                enumerable: !0,
                configurable: !0,
                writable: !0,
              })
            : (obj[key] = value),
          obj
        );
      }
      const resource_manager_logos_debug = browser_default()('ResourceManger');
      class LogoResourceManager {
        constructor(logoResources) {
          resource_manager_logos_defineProperty(this, 'getIconPath', async (logoName) => {
            if (!this.logoExists(logoName)) return null;
            if (!this.logos[logoName].hasOwnProperty('icons')) return null;
            const logoIcon = this.logos[logoName].icons[logoName];
            try {
              return (
                await __webpack_require__('./src/assets eager recursive ^\\.\\/resources.*$')(
                  './resources'.concat(logoIcon),
                )
              ).default;
            } catch {
              return (
                resource_manager_logos_debug('failed to load icon for "'.concat(logoName, '"')),
                null
              );
            }
          }),
            resource_manager_logos_defineProperty(this, 'getHeaderLogoPath', async (theme) => {
              const themeIcon =
                theme === ThemeMode.RmfDark
                  ? await this.getIconPath('darkThemeLogo')
                  : await this.getIconPath('headerLogo');
              return (
                themeIcon ||
                (resource_manager_logos_debug('using default header logo'),
                (
                  await Promise.resolve().then(
                    __webpack_require__.t.bind(
                      __webpack_require__,
                      './src/assets/defaultLogo.png',
                      17,
                    ),
                  )
                ).default)
              );
            }),
            resource_manager_logos_defineProperty(
              this,
              'logoExists',
              (logoName) => !!this.logos.hasOwnProperty(logoName),
            ),
            (this.logos = logoResources);
        }
        get all() {
          return this.logos;
        }
        get allValues() {
          return Object.values(this.logos);
        }
      }
      function resource_manager_robots_defineProperty(obj, key, value) {
        return (
          (key = (function resource_manager_robots_toPropertyKey(t) {
            var i = (function resource_manager_robots_toPrimitive(t, r) {
              if ('object' != typeof t || !t) return t;
              var e = t[Symbol.toPrimitive];
              if (void 0 !== e) {
                var i = e.call(t, r || 'default');
                if ('object' != typeof i) return i;
                throw new TypeError('@@toPrimitive must return a primitive value.');
              }
              return ('string' === r ? String : Number)(t);
            })(t, 'string');
            return 'symbol' == typeof i ? i : i + '';
          })(key)) in obj
            ? Object.defineProperty(obj, key, {
                value,
                enumerable: !0,
                configurable: !0,
                writable: !0,
              })
            : (obj[key] = value),
          obj
        );
      }
      const resource_manager_robots_debug = browser_default()('ResourceManager');
      class RobotResourceManager {
        constructor(robotResources) {
          resource_manager_robots_defineProperty(this, 'getAvailablePlacesPerFleet', (fleetName) =>
            this.fleetExists(fleetName) && this.placesExists(fleetName)
              ? Object.keys(this.robots[fleetName].places)
              : null,
          ),
            resource_manager_robots_defineProperty(
              this,
              'getIconPath',
              async (fleetName, robotModel) => {
                if (!this.fleetExists(fleetName))
                  return (
                    resource_manager_robots_debug(
                      'failed to load icon for "'
                        .concat(fleetName, '/')
                        .concat(robotModel, '" (fleet not in resources)'),
                    ),
                    null
                  );
                if (!this.robots[fleetName].hasOwnProperty('icons'))
                  return (
                    resource_manager_robots_debug(
                      'failed to load icon for "'
                        .concat(fleetName, '/')
                        .concat(robotModel, '" (fleet/model does not have an icon)'),
                    ),
                    null
                  );
                const robotIcons = this.robots[fleetName].icons;
                let iconPath = null;
                iconPath =
                  robotModel && robotIcons.hasOwnProperty(robotModel)
                    ? robotIcons[robotModel]
                    : robotIcons[fleetName]
                      ? ''.concat(robotIcons[fleetName])
                      : null;
                try {
                  return (
                    await __webpack_require__('./src/assets eager recursive ^\\.\\/resources.*$')(
                      './resources'.concat(iconPath),
                    )
                  ).default;
                } catch {
                  return (
                    resource_manager_robots_debug(
                      'failed to load icon for "'
                        .concat(fleetName, '/')
                        .concat(robotModel, '" (failed to load icon module)'),
                    ),
                    null
                  );
                }
              },
            ),
            resource_manager_robots_defineProperty(
              this,
              'getRobotIconScale',
              (fleetName, robotModel) => {
                if (!this.fleetExists(fleetName))
                  return (
                    resource_manager_robots_debug(
                      'failed to load scale for "'
                        .concat(fleetName, '/')
                        .concat(robotModel, '" (fleet not in resources)'),
                    ),
                    null
                  );
                if (!this.robots[fleetName].hasOwnProperty('scale'))
                  return (
                    resource_manager_robots_debug(
                      'failed to load scale for "'
                        .concat(fleetName, '/')
                        .concat(robotModel, '" (fleet/model does not have an scale)'),
                    ),
                    null
                  );
                const robotScale = this.robots[fleetName].scale;
                return robotModel ? robotScale : null;
              },
            ),
            resource_manager_robots_defineProperty(
              this,
              'getDispensersPerFleet',
              (fleetName, placeName) =>
                this.fleetExists(fleetName) && this.placesExists(fleetName)
                  ? this.robots[fleetName].places.hasOwnProperty(placeName)
                    ? this.robots[fleetName].places[placeName]
                    : (resource_manager_robots_debug(
                        'failed to load dispensers for "'
                          .concat(fleetName, ', ')
                          .concat(placeName, '" (place does not exist in resources)'),
                      ),
                      null)
                  : (resource_manager_robots_debug(
                      'failed to load dispensers for "'
                        .concat(fleetName, ', ')
                        .concat(placeName, '" (fleet or place does not exist in resources)'),
                    ),
                    null),
            ),
            resource_manager_robots_defineProperty(
              this,
              'fleetExists',
              (fleetName) => !!this.robots.hasOwnProperty(fleetName),
            ),
            resource_manager_robots_defineProperty(
              this,
              'placesExists',
              (fleetName) => !!this.robots[fleetName].hasOwnProperty('places'),
            ),
            (this.robots = robotResources);
        }
      }
      var _ResourceManager;
      const resource_manager_debug = browser_default()('ResourceManager');
      class ResourceManager {
        constructor(resources) {
          (this.robots = new RobotResourceManager(resources.robots || {})),
            (this.logos = new LogoResourceManager(resources.logos || {})),
            resources.dispensers &&
              (this.dispensers = new DispenserResourceManager(resources.dispensers)),
            (this.helpLink =
              resources.helpLink || 'https://osrf.github.io/ros2multirobotbook/rmf-core.html'),
            (this.reportIssue =
              resources.reportIssue || 'https://github.com/open-rmf/rmf-web/issues');
        }
      }
      (_ResourceManager = ResourceManager),
        (function resource_manager_defineProperty(obj, key, value) {
          return (
            (key = (function resource_manager_toPropertyKey(t) {
              var i = (function resource_manager_toPrimitive(t, r) {
                if ('object' != typeof t || !t) return t;
                var e = t[Symbol.toPrimitive];
                if (void 0 !== e) {
                  var i = e.call(t, r || 'default');
                  if ('object' != typeof i) return i;
                  throw new TypeError('@@toPrimitive must return a primitive value.');
                }
                return ('string' === r ? String : Number)(t);
              })(t, 'string');
              return 'symbol' == typeof i ? i : i + '';
            })(key)) in obj
              ? Object.defineProperty(obj, key, {
                  value,
                  enumerable: !0,
                  configurable: !0,
                  writable: !0,
                })
              : (obj[key] = value),
            obj
          );
        })(ResourceManager, 'defaultResourceManager', async () => {
          try {
            const resources = await __webpack_require__('./src/assets eager recursive ^\\.\\/.*$')(
              './'.concat('resources/main.json'),
            );
            return new _ResourceManager(resources);
          } catch {
            return resource_manager_debug('failed to load resource file'), new _ResourceManager({});
          }
        });
      ''.concat('.', '/login'),
        ''.concat('.', '/tasks'),
        ''.concat('.', '/robots'),
        ''.concat('.', '/admin/*'),
        ''.concat('.', '/custom1'),
        ''.concat('.', '/custom2');
      var process = __webpack_require__(
        '../../node_modules/.pnpm/process@0.11.10/node_modules/process/browser.js',
      );
      (() => {
        if (!process.env.REACT_APP_AUTH_PROVIDER) return new StubAuthenticator();
        const provider = process.env.REACT_APP_AUTH_PROVIDER;
        switch (provider) {
          case 'keycloak':
            if (!process.env.REACT_APP_KEYCLOAK_CONFIG)
              throw new Error('missing REACT_APP_KEYCLOAK_CONFIG');
            return new KeycloakAuthenticator(
              JSON.parse(process.env.REACT_APP_KEYCLOAK_CONFIG),
              ''.concat(window.location.origin).concat('.', '/silent-check-sso.html'),
            );
          case 'stub':
            return new StubAuthenticator();
          default:
            throw new Error('unknown auth provider "'.concat(provider, '"'));
        }
      })(),
        defaultSettings();
      const AppControllerContext = react.createContext({
        updateSettings: () => {},
        showAlert: () => {},
        setExtraAppbarIcons: () => {},
      });
    },
    '../react-components/dist/index.js': (
      __unused_webpack_module,
      __webpack_exports__,
      __webpack_require__,
    ) => {
      'use strict';
      __webpack_require__.d(__webpack_exports__, {
        KK: () => ConfirmationDialog,
        Rh: () => Loading,
        Mq: () => TransferList,
        Yb: () => useAsync,
      });
      __webpack_require__(
        '../../node_modules/.pnpm/@emotion+styled@11.9.3_@babel+core@7.18.6_@emotion+react@11.9.3_@types+react@18.2.14_react@18.2.0/node_modules/@emotion/styled/dist/emotion-styled.browser.esm.js',
      ),
        __webpack_require__(
          '../../node_modules/.pnpm/@fontsource+roboto@4.5.7/node_modules/@fontsource/roboto/300.css',
        ),
        __webpack_require__(
          '../../node_modules/.pnpm/@fontsource+roboto@4.5.7/node_modules/@fontsource/roboto/400.css',
        ),
        __webpack_require__(
          '../../node_modules/.pnpm/@fontsource+roboto@4.5.7/node_modules/@fontsource/roboto/500.css',
        ),
        __webpack_require__(
          '../../node_modules/.pnpm/@fontsource+roboto@4.5.7/node_modules/@fontsource/roboto/700.css',
        );
      var styled = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/styles/styled.js',
        ),
        Tab = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Tab/Tab.js',
        ),
        react = __webpack_require__(
          '../../node_modules/.pnpm/react@18.2.0/node_modules/react/index.js',
        ),
        __assign = function () {
          return (
            (__assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            __assign.apply(this, arguments)
          );
        };
      (0, styled.Ay)(function (props) {
        return react.createElement(Tab.A, __assign({}, props));
      })(function (_a) {
        var theme = _a.theme;
        return {
          color: theme.palette.primary.contrastText,
          opacity: 0.6,
          '&.Mui-selected': { color: theme.palette.primary.contrastText, opacity: 1 },
        };
      });
      var useTheme = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/styles/useTheme.js',
        ),
        Box = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Box/Box.js',
        ),
        LinearProgress = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/LinearProgress/LinearProgress.js',
        ),
        Typography = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Typography/Typography.js',
        ),
        TextField = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/TextField/TextField.js',
        ),
        Divider = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Divider/Divider.js',
        ),
        Button = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Button/Button.js',
        ),
        Dialog = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Dialog/Dialog.js',
        ),
        DialogActions = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/DialogActions/DialogActions.js',
        ),
        DialogContent = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/DialogContent/DialogContent.js',
        ),
        DialogTitle = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/DialogTitle/DialogTitle.js',
        ),
        alert_dialog_assign = function () {
          return (
            (alert_dialog_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            alert_dialog_assign.apply(this, arguments)
          );
        },
        CloseAlertDialog = react.memo(function (props) {
          var title = props.title;
          return react.createElement(Dialog.A, { key: title, open: !1 });
        }),
        AlertDialog = react.memo(function (props) {
          var theme = (0, useTheme.A)(),
            LinearProgressWithLabel = function (props) {
              return react.createElement(
                Box.A,
                { component: 'div', sx: { display: 'flex', alignItems: 'center' } },
                react.createElement(
                  Box.A,
                  { component: 'div', sx: { width: '100%', mr: 1 } },
                  react.createElement(
                    LinearProgress.A,
                    alert_dialog_assign({ variant: 'determinate' }, props, {
                      value: 100 * props.value,
                    }),
                  ),
                ),
                react.createElement(
                  Box.A,
                  { component: 'div', sx: { minWidth: 35 } },
                  react.createElement(
                    Typography.A,
                    { variant: 'body2', color: 'text.secondary' },
                    ''.concat(Math.round(100 * props.value), '%'),
                  ),
                ),
              );
            },
            onDismiss = props.onDismiss,
            onAcknowledge = props.onAcknowledge,
            onInspect = props.onInspect,
            acknowledgedBy = props.acknowledgedBy,
            title = props.title,
            progress = props.progress,
            alertContents = props.alertContents,
            backgroundColor = props.backgroundColor,
            _a = react.useState(!0),
            isOpen = _a[0],
            setIsOpen = _a[1],
            _b = react.useState(void 0 !== acknowledgedBy),
            acknowledged = _b[0],
            setAcknowledged = _b[1];
          return react.createElement(
            Dialog.A,
            {
              PaperProps: { style: { backgroundColor, boxShadow: 'none' } },
              maxWidth: 'sm',
              fullWidth: !0,
              open: isOpen,
              key: title,
            },
            react.createElement(DialogTitle.A, { align: 'center' }, title),
            react.createElement(Divider.A, null),
            progress
              ? react.createElement(
                  react.Fragment,
                  null,
                  react.createElement(
                    Typography.A,
                    { variant: 'body2', fontWeight: 'bold', ml: 3, mt: 1 },
                    'Task progress',
                  ),
                  react.createElement(
                    Box.A,
                    { component: 'div', width: 0.95, ml: 3 },
                    react.createElement(LinearProgressWithLabel, { value: progress }),
                  ),
                )
              : null,
            react.createElement(
              DialogContent.A,
              null,
              (function (alertContents) {
                return react.createElement(
                  react.Fragment,
                  null,
                  alertContents.map(function (message, index) {
                    return react.createElement(
                      'div',
                      { key: index },
                      react.createElement(TextField.A, {
                        label: message.title,
                        id: 'standard-size-small',
                        size: 'small',
                        variant: 'filled',
                        sx: { background: theme.palette.background.default, pointerEvents: 'none' },
                        fullWidth: !0,
                        multiline: !0,
                        maxRows: 4,
                        margin: 'dense',
                        value: message.value,
                      }),
                    );
                  }),
                );
              })(alertContents),
            ),
            react.createElement(
              DialogActions.A,
              null,
              onInspect
                ? react.createElement(
                    Button.A,
                    {
                      size: 'small',
                      variant: 'contained',
                      onClick: onInspect,
                      disabled: !1,
                      autoFocus: !0,
                    },
                    'Inspect',
                  )
                : null,
              acknowledged
                ? react.createElement(
                    Button.A,
                    { size: 'small', variant: 'contained', disabled: !0, autoFocus: !0 },
                    acknowledgedBy ? 'Acknowledged by '.concat(acknowledgedBy) : 'Acknowledged',
                  )
                : void 0 === onAcknowledge
                  ? null
                  : react.createElement(
                      Button.A,
                      {
                        size: 'small',
                        variant: 'contained',
                        onClick: function () {
                          setAcknowledged(!0), onAcknowledge();
                        },
                        disabled: !1,
                        autoFocus: !0,
                      },
                      'Acknowledge',
                    ),
              react.createElement(
                Button.A,
                {
                  size: 'small',
                  variant: 'contained',
                  onClick: function () {
                    setIsOpen(!1), onDismiss();
                  },
                  autoFocus: !0,
                },
                acknowledged ? 'Close' : 'Dismiss',
              ),
            ),
          );
        });
      (CloseAlertDialog.__docgenInfo = {
        description: '',
        methods: [],
        displayName: 'CloseAlertDialog',
      }),
        (AlertDialog.__docgenInfo = { description: '', methods: [], displayName: 'AlertDialog' });
      var DataGrid = __webpack_require__(
        '../../node_modules/.pnpm/@mui+x-data-grid@5.17.26_@mui+material@5.8.7_@mui+system@5.8.7_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/x-data-grid/DataGrid/DataGrid.js',
      );
      var crc = __webpack_require__('../../node_modules/.pnpm/crc@3.8.0/node_modules/crc/index.js'),
        browser = __webpack_require__(
          '../../node_modules/.pnpm/node-vibrant@3.1.6/node_modules/node-vibrant/lib/browser.js',
        ),
        browser_default = __webpack_require__.n(browser),
        Grid = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Grid/Grid.js',
        ),
        ReactCustomizableProgressbar = __webpack_require__(
          '../../node_modules/.pnpm/react-customizable-progressbar@1.2.0_react@18.2.0/node_modules/react-customizable-progressbar/dist/ReactCustomizableProgressbar.js',
        ),
        ReactCustomizableProgressbar_default = __webpack_require__.n(ReactCustomizableProgressbar),
        circular_progress_bar_assign = function () {
          return (
            (circular_progress_bar_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            circular_progress_bar_assign.apply(this, arguments)
          );
        },
        classes = { indicator: 'circular-progressbar-indicator' },
        StyledCircularProgressBar = (0, styled.Ay)(function (props) {
          return react.createElement(
            ReactCustomizableProgressbar_default(),
            circular_progress_bar_assign({}, props),
          );
        })(function () {
          var _a;
          return (
            ((_a = {})['& .'.concat(classes.indicator)] = {
              display: 'flex',
              flexDirection: 'column',
              justifyContent: 'center',
              textAlign: 'center',
              position: 'absolute',
              top: 0,
              width: '100%',
              height: '100%',
              margin: '0 auto',
            }),
            _a
          );
        });
      function CircularProgressBar(props) {
        var progress = props.progress,
          strokeColor = props.strokeColor,
          children = props.children;
        return react.createElement(
          StyledCircularProgressBar,
          {
            radius: 60,
            progress,
            cut: 120,
            rotate: -210,
            strokeColor,
            strokeWidth: 10,
            trackStrokeWidth: 10,
          },
          react.createElement('div', { className: classes.indicator }, children),
        );
      }
      CircularProgressBar.__docgenInfo = {
        description: '',
        methods: [],
        displayName: 'CircularProgressBar',
      };
      var linear_progress_bar_assign = function () {
        return (
          (linear_progress_bar_assign =
            Object.assign ||
            function (t) {
              for (var s, i = 1, n = arguments.length; i < n; i++)
                for (var p in (s = arguments[i]))
                  Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
              return t;
            }),
          linear_progress_bar_assign.apply(this, arguments)
        );
      };
      function LinearProgressBar(props) {
        return react.createElement(
          Box.A,
          { component: 'div', display: 'flex', alignItems: 'center' },
          react.createElement(
            Box.A,
            { component: 'div', width: '100%', mr: 1 },
            react.createElement(
              LinearProgress.A,
              linear_progress_bar_assign({ color: 'secondary', variant: 'determinate' }, props),
            ),
          ),
          react.createElement(
            Box.A,
            { component: 'div', minWidth: 35 },
            react.createElement(
              Typography.A,
              { variant: 'body2' },
              ''.concat(Math.floor(props.value), '%'),
            ),
          ),
        );
      }
      LinearProgressBar.__docgenInfo = {
        description: '',
        methods: [],
        displayName: 'LinearProgressBar',
      };
      var robot_info_classes_button = 'robot-info-button';
      (0, styled.Ay)('div')(function () {
        var _a;
        return (
          ((_a = {})['& .'.concat(robot_info_classes_button)] = {
            '&:hover': { background: 'none', cursor: 'default' },
          }),
          _a
        );
      });
      var TableRow = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/TableRow/TableRow.js',
        ),
        TableCell = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/TableCell/TableCell.js',
        ),
        Table = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Table/Table.js',
        ),
        TableHead = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/TableHead/TableHead.js',
        ),
        TableBody = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/TableBody/TableBody.js',
        );
      var lib = __webpack_require__('../api-client/dist/lib/index.js'),
        dist = __webpack_require__('../rmf-models/dist/index.js');
      var __awaiter = function (thisArg, _arguments, P, generator) {
          return new (P || (P = Promise))(function (resolve, reject) {
            function fulfilled(value) {
              try {
                step(generator.next(value));
              } catch (e) {
                reject(e);
              }
            }
            function rejected(value) {
              try {
                step(generator.throw(value));
              } catch (e) {
                reject(e);
              }
            }
            function step(result) {
              result.done
                ? resolve(result.value)
                : (function adopt(value) {
                    return value instanceof P
                      ? value
                      : new P(function (resolve) {
                          resolve(value);
                        });
                  })(result.value).then(fulfilled, rejected);
            }
            step((generator = generator.apply(thisArg, _arguments || [])).next());
          });
        },
        __generator = function (thisArg, body) {
          var f,
            y,
            t,
            g,
            _ = {
              label: 0,
              sent: function () {
                if (1 & t[0]) throw t[1];
                return t[1];
              },
              trys: [],
              ops: [],
            };
          return (
            (g = { next: verb(0), throw: verb(1), return: verb(2) }),
            'function' == typeof Symbol &&
              (g[Symbol.iterator] = function () {
                return this;
              }),
            g
          );
          function verb(n) {
            return function (v) {
              return (function step(op) {
                if (f) throw new TypeError('Generator is already executing.');
                for (; g && ((g = 0), op[0] && (_ = 0)), _; )
                  try {
                    if (
                      ((f = 1),
                      y &&
                        (t =
                          2 & op[0]
                            ? y.return
                            : op[0]
                              ? y.throw || ((t = y.return) && t.call(y), 0)
                              : y.next) &&
                        !(t = t.call(y, op[1])).done)
                    )
                      return t;
                    switch (((y = 0), t && (op = [2 & op[0], t.value]), op[0])) {
                      case 0:
                      case 1:
                        t = op;
                        break;
                      case 4:
                        return _.label++, { value: op[1], done: !1 };
                      case 5:
                        _.label++, (y = op[1]), (op = [0]);
                        continue;
                      case 7:
                        (op = _.ops.pop()), _.trys.pop();
                        continue;
                      default:
                        if (
                          !((t = _.trys),
                          (t = t.length > 0 && t[t.length - 1]) || (6 !== op[0] && 2 !== op[0]))
                        ) {
                          _ = 0;
                          continue;
                        }
                        if (3 === op[0] && (!t || (op[1] > t[0] && op[1] < t[3]))) {
                          _.label = op[1];
                          break;
                        }
                        if (6 === op[0] && _.label < t[1]) {
                          (_.label = t[1]), (t = op);
                          break;
                        }
                        if (t && _.label < t[2]) {
                          (_.label = t[2]), _.ops.push(op);
                          break;
                        }
                        t[2] && _.ops.pop(), _.trys.pop();
                        continue;
                    }
                    op = body.call(thisArg, _);
                  } catch (e) {
                    (op = [6, e]), (y = 0);
                  } finally {
                    f = t = 0;
                  }
                if (5 & op[0]) throw op[1];
                return { value: op[0] ? op[1] : void 0, done: !0 };
              })([n, v]);
            };
          }
        };
      function _hash(s) {
        return (0, crc.QR)(s);
      }
      var ColorManager = (function () {
        function ColorManager() {
          (this.conflictHighlight = '#f44336'), (this._robotColorCache = {});
        }
        return (
          (ColorManager.prototype.robotPrimaryColor = function (fleet, name, model, image) {
            return __awaiter(this, void 0, void 0, function () {
              var key, palette, rgb, colorHolder, e_1, _a;
              return __generator(this, function (_b) {
                switch (_b.label) {
                  case 0:
                    return (
                      (key = (function robotHash(name, fleet) {
                        return ''.concat(name, '__').concat(fleet);
                      })(name, fleet)),
                      this._robotColorCache[key]
                        ? [2, this._robotColorCache[key]]
                        : image
                          ? [3, 1]
                          : ((this._robotColorCache[key] = this._robotColorFromId(
                              fleet,
                              name,
                              model,
                            )),
                            [2, this._robotColorCache[key]])
                    );
                  case 1:
                    return (
                      _b.trys.push([1, 3, , 4]), [4, browser_default().from(image).getSwatches()]
                    );
                  case 2:
                    return (
                      (palette = _b.sent()),
                      (rgb =
                        null === (_a = palette.Vibrant) || void 0 === _a ? void 0 : _a.getRgb())
                        ? ((colorHolder = 'rgb('
                            .concat(rgb[0], ', ')
                            .concat(rgb[1], ', ')
                            .concat(rgb[2], ')')),
                          (this._robotColorCache[key] = colorHolder),
                          [2, colorHolder])
                        : [3, 4]
                    );
                  case 3:
                    return (
                      (e_1 = _b.sent()),
                      console.warn(
                        'unable to get color from image, falling back to color from id ('.concat(
                          e_1.message,
                          ')',
                        ),
                      ),
                      [3, 4]
                    );
                  case 4:
                    return [2, this.robotPrimaryColor(fleet, name, model)];
                }
              });
            });
          }),
          (ColorManager._getLightColor = function (firstNumber, secondNumber) {
            var satlum = secondNumber % 2500,
              saturation = 50 + (satlum % 50),
              luminance = 25 + satlum / 50;
            return 'hsl('
              .concat(50 + (firstNumber % 220), ', ')
              .concat(saturation, '%, ')
              .concat(luminance, '%)');
          }),
          (ColorManager.prototype._robotColorFromId = function (fleet, name, model) {
            var modelHash = _hash(model),
              nameHash = _hash(name);
            return ColorManager._getLightColor(modelHash, nameHash);
          }),
          ColorManager
        );
      })();
      var ColorContext = react.createContext(new ColorManager()),
        clsx_m = __webpack_require__(
          '../../node_modules/.pnpm/clsx@1.2.1/node_modules/clsx/dist/clsx.m.js',
        ),
        CircularProgress = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/CircularProgress/CircularProgress.js',
        ),
        loading_assign = function () {
          return (
            (loading_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            loading_assign.apply(this, arguments)
          );
        },
        loading_rest = function (s, e) {
          var t = {};
          for (var p in s)
            Object.prototype.hasOwnProperty.call(s, p) && e.indexOf(p) < 0 && (t[p] = s[p]);
          if (null != s && 'function' == typeof Object.getOwnPropertySymbols) {
            var i = 0;
            for (p = Object.getOwnPropertySymbols(s); i < p.length; i++)
              e.indexOf(p[i]) < 0 &&
                Object.prototype.propertyIsEnumerable.call(s, p[i]) &&
                (t[p[i]] = s[p[i]]);
          }
          return t;
        },
        loading_classes = {
          root: 'loading-root',
          container: 'loading-container',
          loadingProgress: 'loading-progress',
          loadingOverlay: 'loading-overlay',
        },
        loading_StyledDiv = (0, styled.Ay)('div')(function () {
          var _a;
          return (
            ((_a = {})['&.'.concat(loading_classes.root)] = {
              position: 'relative',
              height: '100%',
              flex: '1 1 auto',
            }),
            (_a['& .'.concat(loading_classes.container)] = {
              position: 'absolute',
              display: 'flex',
              justifyContent: 'center',
              alignItems: 'center',
              left: 0,
              top: 0,
              width: '100%',
              height: '100%',
            }),
            (_a['& .'.concat(loading_classes.loadingProgress)] = {
              position: 'absolute',
              flex: '0 0 auto',
            }),
            (_a['& .'.concat(loading_classes.loadingOverlay)] = {
              filter: 'blur(2px)',
              opacity: 0.6,
              pointerEvents: 'none',
              userSelect: 'none',
            }),
            _a
          );
        });
      function Loading(_a) {
        var children = _a.children,
          _b = _a.loading,
          loading = void 0 !== _b && _b,
          _c = _a.hideChildren,
          hideChildren = void 0 !== _c && _c,
          _d = _a.loadingClassName,
          loadingClassName = void 0 === _d ? '' : _d,
          _e = _a.className,
          className = void 0 === _e ? '' : _e,
          _f = _a.style,
          style = void 0 === _f ? {} : _f,
          otherProps = loading_rest(_a, [
            'children',
            'loading',
            'hideChildren',
            'loadingClassName',
            'className',
            'style',
          ]);
        return loading
          ? react.createElement(
              loading_StyledDiv,
              { className: loading_classes.root },
              react.createElement(
                'div',
                {
                  className: ''
                    .concat(loading_classes.loadingOverlay, ' ')
                    .concat(loadingClassName),
                  style: { visibility: hideChildren ? 'hidden' : 'visible' },
                },
                children,
              ),
              react.createElement(
                'div',
                { className: loading_classes.container },
                react.createElement(
                  CircularProgress.A,
                  loading_assign(
                    {
                      'aria-label': 'loading',
                      className: ''.concat(loading_classes.loadingProgress, ' ').concat(className),
                      style: loading_assign({ visibility: loading ? 'visible' : 'hidden' }, style),
                    },
                    otherProps,
                  ),
                ),
              ),
            )
          : react.createElement(react.Fragment, null, children);
      }
      Loading.__docgenInfo = { description: '', methods: [], displayName: 'Loading' };
      var confirmation_dialog_assign = function () {
          return (
            (confirmation_dialog_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            confirmation_dialog_assign.apply(this, arguments)
          );
        },
        confirmation_dialog_rest = function (s, e) {
          var t = {};
          for (var p in s)
            Object.prototype.hasOwnProperty.call(s, p) && e.indexOf(p) < 0 && (t[p] = s[p]);
          if (null != s && 'function' == typeof Object.getOwnPropertySymbols) {
            var i = 0;
            for (p = Object.getOwnPropertySymbols(s); i < p.length; i++)
              e.indexOf(p[i]) < 0 &&
                Object.prototype.propertyIsEnumerable.call(s, p[i]) &&
                (t[p[i]] = s[p[i]]);
          }
          return t;
        },
        dialogClasses = {
          title: 'confirmation-dialogue-info-value',
          actionBtn: 'confirmation-dialogue-action-button',
        },
        StyledDialog = (0, styled.Ay)(function (props) {
          return react.createElement(Dialog.A, confirmation_dialog_assign({}, props));
        })(function () {
          var _a;
          return (
            ((_a = {})['& .'.concat(dialogClasses.title)] = { flex: '1 1 auto' }),
            (_a['& .'.concat(dialogClasses.actionBtn)] = { minWidth: 80 }),
            _a
          );
        });
      function ConfirmationDialog(_a) {
        var _b = _a.title,
          title = void 0 === _b ? 'Confirm' : _b,
          _c = _a.confirmText,
          confirmText = void 0 === _c ? 'OK' : _c,
          _d = _a.cancelText,
          cancelText = void 0 === _d ? 'Cancel' : _d,
          _e = _a.submitting,
          submitting = void 0 !== _e && _e,
          classes = _a.classes,
          onSubmit = _a.onSubmit,
          toolbar = _a.toolbar,
          onClose = _a.onClose,
          children = _a.children,
          otherProps = confirmation_dialog_rest(_a, [
            'title',
            'confirmText',
            'cancelText',
            'submitting',
            'classes',
            'onSubmit',
            'toolbar',
            'onClose',
            'children',
          ]);
        return react.createElement(
          StyledDialog,
          confirmation_dialog_assign({ onClose }, otherProps),
          react.createElement(
            'form',
            {
              onSubmit: function (ev) {
                ev.preventDefault(), onSubmit && onSubmit(ev);
              },
              'aria-label': title,
            },
            react.createElement(
              DialogTitle.A,
              null,
              react.createElement(
                Grid.Ay,
                { container: !0, wrap: 'nowrap' },
                react.createElement(Grid.Ay, { item: !0, className: dialogClasses.title }, title),
                react.createElement(Grid.Ay, { item: !0 }, toolbar),
              ),
            ),
            react.createElement(DialogContent.A, null, children),
            react.createElement(
              DialogActions.A,
              null,
              react.createElement(
                Button.A,
                {
                  variant: 'outlined',
                  onClick: function (ev) {
                    return onClose && onClose(ev, 'escapeKeyDown');
                  },
                  disabled: submitting,
                  className: (0, clsx_m.default)(
                    dialogClasses.actionBtn,
                    null == classes ? void 0 : classes.button,
                  ),
                },
                cancelText,
              ),
              react.createElement(
                Button.A,
                {
                  variant: 'contained',
                  type: 'submit',
                  color: 'primary',
                  disabled: submitting,
                  className: (0, clsx_m.default)(
                    dialogClasses.actionBtn,
                    null == classes ? void 0 : classes.button,
                  ),
                },
                react.createElement(
                  Loading,
                  { hideChildren: !0, loading: submitting, size: '1.5em', color: 'inherit' },
                  confirmText,
                ),
              ),
            ),
          ),
        );
      }
      ConfirmationDialog.__docgenInfo = {
        description: '',
        methods: [],
        displayName: 'ConfirmationDialog',
      };
      var DoorType,
        DoorMotion,
        DoorMode,
        Card = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Card/Card.js',
        );
      __webpack_require__(
        '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/CardContent/CardContent.js',
      );
      !(function (DoorType) {
        (DoorType[(DoorType.SingleSwing = dist.Door.DOOR_TYPE_SINGLE_SWING)] = 'SingleSwing'),
          (DoorType[(DoorType.SingleSliding = dist.Door.DOOR_TYPE_SINGLE_SLIDING)] =
            'SingleSliding'),
          (DoorType[(DoorType.SingleTelescope = dist.Door.DOOR_TYPE_SINGLE_TELESCOPE)] =
            'SingleTelescope'),
          (DoorType[(DoorType.DoubleSwing = dist.Door.DOOR_TYPE_DOUBLE_SWING)] = 'DoubleSwing'),
          (DoorType[(DoorType.DoubleSliding = dist.Door.DOOR_TYPE_DOUBLE_SLIDING)] =
            'DoubleSliding'),
          (DoorType[(DoorType.DoubleTelescope = dist.Door.DOOR_TYPE_DOUBLE_TELESCOPE)] =
            'DoubleTelescope');
      })(DoorType || (DoorType = {})),
        (function (DoorMotion) {
          (DoorMotion[(DoorMotion.Clockwise = 1)] = 'Clockwise'),
            (DoorMotion[(DoorMotion.AntiClockwise = -1)] = 'AntiClockwise');
        })(DoorMotion || (DoorMotion = {})),
        (function (DoorMode) {
          (DoorMode[(DoorMode.Open = dist.DoorMode.MODE_OPEN)] = 'Open'),
            (DoorMode[(DoorMode.Closed = dist.DoorMode.MODE_CLOSED)] = 'Closed'),
            (DoorMode[(DoorMode.Moving = dist.DoorMode.MODE_MOVING)] = 'Moving');
        })(DoorMode || (DoorMode = {}));
      var HealthStatus;
      __webpack_require__(
        '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/ButtonGroup/ButtonGroup.js',
      );
      !(function (HealthStatus) {
        (HealthStatus.Healthy = 'Healthy'),
          (HealthStatus.Unhealthy = 'Unhealthy'),
          (HealthStatus.Dead = 'Dead');
      })(HealthStatus || (HealthStatus = {}));
      var item_table_assign = function () {
          return (
            (item_table_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            item_table_assign.apply(this, arguments)
          );
        },
        commonStyles = { overflow: 'hidden', height: 31 },
        useFixedTableCellStylesClasses = {
          fixedTableCell: ''.concat('item-table-cell', '-fix-cell'),
          fixedLastTableCell: ''.concat('item-table-cell', '-last-fix-cell'),
        },
        ItemTableCell = (0, styled.Ay)(function (props) {
          return react.createElement(TableCell.A, item_table_assign({}, props));
        })(function () {
          var _a;
          return (
            ((_a = {})['&.'.concat(useFixedTableCellStylesClasses.fixedTableCell)] =
              item_table_assign({ flex: '1 0 0' }, commonStyles)),
            (_a['&.'.concat(useFixedTableCellStylesClasses.fixedLastTableCell)] = item_table_assign(
              { flex: '1.5 0 0' },
              commonStyles,
            )),
            _a
          );
        });
      __webpack_require__(
        '../../node_modules/.pnpm/shallowequal@1.1.0/node_modules/shallowequal/index.js',
      );
      var id = 0;
      function uniqueId() {
        return (id++).toString();
      }
      var icons_material_Error = __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/Error.js',
        ),
        error_overlay_classes_errorIcon = 'erroroverlay-error-icon',
        error_overlay_classes_errorMsg = 'erroroverlay-error-msg',
        error_overlay_classes_errorDisabled = 'erroroverlay-error-disabled',
        error_overlay_classes_overlay = 'erroroverlay-overlay',
        error_overlay_classes_container = 'erroroverlay-container',
        error_overlay_classes_disableSelect = 'erroroverlay-disable-select',
        error_overlay_StyledDiv = (0, styled.Ay)('div')(function (_a) {
          var _b,
            theme = _a.theme;
          return (
            ((_b = {})['& .'.concat(error_overlay_classes_errorIcon)] = {
              color: theme.palette.error.main,
              fontSize: '2rem',
            }),
            (_b['& .'.concat(error_overlay_classes_errorMsg)] = { margin: '0.5rem' }),
            (_b['& .'.concat(error_overlay_classes_errorDisabled)] = {
              pointerEvents: 'none',
              filter: 'blur(.25rem)',
              gridArea: '1 / 1',
              opacity: 0.6,
            }),
            (_b['& .'.concat(error_overlay_classes_overlay)] = {
              gridArea: '1 / 1',
              backdropFilter: 'blur(.5rem)',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
            }),
            (_b['&.'.concat(error_overlay_classes_container)] = { display: 'grid' }),
            (_b['& .'.concat(error_overlay_classes_disableSelect)] = { userSelect: 'none' }),
            _b
          );
        });
      react.memo(function (props) {
        var errorMsg = props.errorMsg,
          children = props.children,
          overrideErrorStyle = props.overrideErrorStyle;
        return errorMsg
          ? react.createElement(
              error_overlay_StyledDiv,
              { className: error_overlay_classes_container },
              react.createElement(
                'div',
                { className: error_overlay_classes_errorDisabled },
                children,
              ),
              react.createElement(
                'div',
                {
                  className: children
                    ? ''
                        .concat(error_overlay_classes_overlay, ' ')
                        .concat(error_overlay_classes_disableSelect)
                    : error_overlay_classes_disableSelect,
                },
                react.createElement(
                  'div',
                  null,
                  react.createElement(
                    Grid.Ay,
                    {
                      container: !0,
                      direction: 'row',
                      justifyContent: 'center',
                      alignItems: 'center',
                      spacing: 2,
                    },
                    react.createElement(
                      Grid.Ay,
                      { item: !0 },
                      react.createElement(icons_material_Error.A, {
                        className: error_overlay_classes_errorIcon,
                      }),
                    ),
                    react.createElement(
                      Grid.Ay,
                      { item: !0 },
                      react.createElement(
                        Typography.A,
                        { color: 'error', variant: 'h4', align: 'center' },
                        'Error',
                      ),
                    ),
                  ),
                  react.createElement(
                    Typography.A,
                    {
                      className: overrideErrorStyle || error_overlay_classes_errorMsg,
                      color: 'error',
                      variant: 'h6',
                      align: 'center',
                    },
                    errorMsg,
                  ),
                ),
              ),
            )
          : react.createElement(react.Fragment, null, children);
      }).__docgenInfo = { description: '', methods: [], displayName: 'ErrorOverlay' };
      var form_inputs_assign = function () {
          return (
            (form_inputs_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            form_inputs_assign.apply(this, arguments)
          );
        },
        form_inputs_rest = function (s, e) {
          var t = {};
          for (var p in s)
            Object.prototype.hasOwnProperty.call(s, p) && e.indexOf(p) < 0 && (t[p] = s[p]);
          if (null != s && 'function' == typeof Object.getOwnPropertySymbols) {
            var i = 0;
            for (p = Object.getOwnPropertySymbols(s); i < p.length; i++)
              e.indexOf(p[i]) < 0 &&
                Object.prototype.propertyIsEnumerable.call(s, p[i]) &&
                (t[p[i]] = s[p[i]]);
          }
          return t;
        };
      function PositiveIntField(_a) {
        var _b = _a.value,
          value = void 0 === _b ? 0 : _b,
          onChange = _a.onChange,
          props = form_inputs_rest(_a, ['value', 'onChange']),
          _c = react.useState(value.toString()),
          valueInput = _c[0],
          setValueInput = _c[1];
        return (
          react.useEffect(
            function () {
              setValueInput(value.toString());
            },
            [value],
          ),
          react.createElement(
            TextField.A,
            form_inputs_assign({}, props, {
              type: 'number',
              value: valueInput,
              inputProps: { min: 1 },
              onKeyDown: function (ev) {
                '-+.'.indexOf(ev.key) >= 0 && ev.preventDefault(),
                  props.onKeyDown && props.onKeyDown(ev);
              },
              onChange: function (ev) {
                var int = parseInt(ev.target.value);
                int > 0 && onChange && onChange(ev, int), setValueInput(ev.target.value);
              },
            }),
          )
        );
      }
      PositiveIntField.__docgenInfo = {
        description: '',
        methods: [],
        displayName: 'PositiveIntField',
      };
      var _a,
        _b,
        AppBar = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/AppBar/AppBar.js',
        ),
        header_bar_assign = function () {
          return (
            (header_bar_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            header_bar_assign.apply(this, arguments)
          );
        },
        header_bar_classes_root = 'header-bar-root';
      (0, styled.Ay)(function (props) {
        return react.createElement(AppBar.A, header_bar_assign({}, props));
      })(function () {
        var _a;
        return (
          ((_a = {})['&.'.concat(header_bar_classes_root)] = {
            display: 'flex',
            flexDirection: 'row',
            width: '100%',
          }),
          _a
        );
      }),
        __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/ArrowDownward.js',
        ),
        __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/ArrowUpward.js',
        );
      var requestModes = [
          dist.LiftRequest.REQUEST_AGV_MODE,
          dist.LiftRequest.REQUEST_HUMAN_MODE,
          dist.LiftRequest.REQUEST_END_SESSION,
        ],
        requestModeStrings =
          (((_a = {})[dist.LiftRequest.REQUEST_END_SESSION] = 'End Session'),
          (_a[dist.LiftRequest.REQUEST_AGV_MODE] = 'AGV'),
          (_a[dist.LiftRequest.REQUEST_HUMAN_MODE] = 'Human'),
          _a);
      var requestDoorModes = [dist.LiftRequest.DOOR_OPEN, dist.LiftRequest.DOOR_CLOSED],
        requestDoorModeStrings =
          (((_b = {})[dist.LiftRequest.DOOR_OPEN] = 'Open'),
          (_b[dist.LiftRequest.DOOR_CLOSED] = 'Closed'),
          _b);
      var Autocomplete = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Autocomplete/Autocomplete.js',
        ),
        lift_request_dialog_assign = function () {
          return (
            (lift_request_dialog_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            lift_request_dialog_assign.apply(this, arguments)
          );
        },
        lift_request_dialog_classes_closeButton = 'lift-request-close-button',
        lift_request_dialog_classes_form = 'lift-request-form',
        lift_request_dialog_classes_divForm = 'lift-request-divform',
        lift_request_dialog_classes_error = 'lift-request-error',
        lift_request_dialog_classes_input = 'lift-request-input',
        lift_request_dialog_classes_button = 'lift-request-button',
        lift_request_dialog_classes_buttonContainer = 'lift-request-button-container',
        lift_request_dialog_classes_dialogContent = 'lift-request-dialog-content',
        StyledConfirmationDialog = (0, styled.Ay)(function (props) {
          return react.createElement(ConfirmationDialog, lift_request_dialog_assign({}, props));
        })(function (_a) {
          var _b,
            theme = _a.theme;
          return (
            ((_b = {})['& .'.concat(lift_request_dialog_classes_closeButton)] = {
              position: 'absolute',
              right: theme.spacing(1),
              top: theme.spacing(1),
              color: theme.palette.error.main,
            }),
            (_b['& .'.concat(lift_request_dialog_classes_form)] = {
              display: 'flex',
              alignItems: 'center',
              flexDirection: 'column',
              padding: '0.5rem',
            }),
            (_b['& .'.concat(lift_request_dialog_classes_divForm)] = {
              padding: '0.5rem',
              width: '100%',
            }),
            (_b['& .'.concat(lift_request_dialog_classes_error)] = {
              color: theme.palette.error.main,
            }),
            (_b['& .'.concat(lift_request_dialog_classes_input)] = { width: '100%' }),
            (_b['& .'.concat(lift_request_dialog_classes_button)] = { width: '100%' }),
            (_b['& .'.concat(lift_request_dialog_classes_buttonContainer)] = {
              paddingTop: '0.5rem',
              width: '100%',
            }),
            (_b['& .'.concat(lift_request_dialog_classes_dialogContent)] = {
              padding: theme.spacing(5),
            }),
            _b
          );
        }),
        LiftRequestDialog = function (_a) {
          var currentLevel = _a.currentLevel,
            availableLevels = _a.availableLevels,
            availableRequestTypes = _a.availableRequestTypes,
            availableDoorModes = _a.availableDoorModes,
            showFormDialog = _a.showFormDialog,
            onRequestSubmit = _a.onRequestSubmit,
            onClose = _a.onClose,
            _b = react.useState(availableDoorModes[0]),
            doorState = _b[0],
            setDoorState = _b[1],
            _c = react.useState(availableRequestTypes[0]),
            requestType = _c[0],
            setRequestType = _c[1],
            _d = react.useState(currentLevel),
            destination = _d[0],
            setDestination = _d[1],
            _e = react.useState(''),
            doorStateError = _e[0],
            setDoorStateError = _e[1],
            _f = react.useState(''),
            requestTypeError = _f[0],
            setRequestTypeError = _f[1],
            _g = react.useState(''),
            destinationError = _g[0],
            setDestinationError = _g[1],
            isFormValid = function () {
              var isValid = !0;
              return (
                setDoorStateError(''),
                setRequestTypeError(''),
                setDestinationError(''),
                null === requestType &&
                  (setRequestTypeError('Request type cannot be empty'), (isValid = !1)),
                null === doorState &&
                  (setDoorStateError('Door state cannot be empty'), (isValid = !1)),
                destination || (setDestinationError('Destination cannot be empty'), (isValid = !1)),
                isValid
              );
            };
          return react.createElement(
            StyledConfirmationDialog,
            {
              open: showFormDialog,
              onClose: function () {
                return onClose();
              },
              fullWidth: !0,
              maxWidth: 'md',
              onSubmit: function (event) {
                event.preventDefault(),
                  isFormValid() &&
                    (onRequestSubmit &&
                      null !== doorState &&
                      null !== requestType &&
                      onRequestSubmit(event, doorState, requestType, destination),
                    onClose());
              },
              title: 'Lift Request Form',
              confirmText: 'Request',
              cancelText: 'Close',
            },
            react.createElement(
              'div',
              { className: lift_request_dialog_classes_divForm },
              react.createElement(Autocomplete.A, {
                getOptionLabel: function (option) {
                  return option;
                },
                onChange: function (_, value) {
                  return setDestination(value || '');
                },
                options: availableLevels,
                renderInput: function (params) {
                  return react.createElement(
                    TextField.A,
                    lift_request_dialog_assign({}, params, {
                      label: 'Pick a Destination',
                      placeholder: 'Pick a Destination',
                      variant: 'outlined',
                      error: !!destinationError,
                      helperText: destinationError,
                    }),
                  );
                },
                value: destination,
              }),
            ),
            react.createElement(
              'div',
              { className: lift_request_dialog_classes_divForm },
              react.createElement(Autocomplete.A, {
                getOptionLabel: function (option) {
                  return (function requestDoorModeToString(requestDoorMode) {
                    return requestDoorModeStrings[requestDoorMode] || 'Unknown';
                  })(option);
                },
                onChange: function (_, value) {
                  return setDoorState(value);
                },
                options: availableDoorModes,
                renderInput: function (params) {
                  return react.createElement(
                    TextField.A,
                    lift_request_dialog_assign({}, params, {
                      label: 'Pick a Door State',
                      placeholder: 'Pick a Door State',
                      variant: 'outlined',
                      error: !!doorStateError,
                      helperText: doorStateError,
                    }),
                  );
                },
                value: doorState,
              }),
            ),
            react.createElement(
              'div',
              { className: lift_request_dialog_classes_divForm },
              react.createElement(Autocomplete.A, {
                getOptionLabel: function (option) {
                  return (function requestModeToString(requestMode) {
                    return requestModeStrings[requestMode] || 'Unknown ('.concat(requestMode, ')');
                  })(option);
                },
                onChange: function (_, value) {
                  return setRequestType(value);
                },
                options: availableRequestTypes,
                renderInput: function (params) {
                  return react.createElement(
                    TextField.A,
                    lift_request_dialog_assign({}, params, {
                      label: 'Pick Request Type',
                      placeholder: 'Pick Request Type',
                      variant: 'outlined',
                      error: !!requestTypeError,
                      helperText: requestTypeError,
                    }),
                  );
                },
                value: requestType,
              }),
            ),
          );
        };
      LiftRequestDialog.__docgenInfo = {
        description: '',
        methods: [],
        displayName: 'LiftRequestDialog',
      };
      var lift_controls_assign = function () {
          return (
            (lift_controls_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            lift_controls_assign.apply(this, arguments)
          );
        },
        lift_controls_rest = function (s, e) {
          var t = {};
          for (var p in s)
            Object.prototype.hasOwnProperty.call(s, p) && e.indexOf(p) < 0 && (t[p] = s[p]);
          if (null != s && 'function' == typeof Object.getOwnPropertySymbols) {
            var i = 0;
            for (p = Object.getOwnPropertySymbols(s); i < p.length; i++)
              e.indexOf(p[i]) < 0 &&
                Object.prototype.propertyIsEnumerable.call(s, p[i]) &&
                (t[p[i]] = s[p[i]]);
          }
          return t;
        };
      function LiftControls(_a) {
        var currentLevel = _a.currentLevel,
          onClose = _a.onClose,
          otherProps = lift_controls_rest(_a, ['currentLevel', 'onClose']),
          _b = react.useState(!1),
          showDialog = _b[0],
          setShowDialog = _b[1],
          _c = react.useState(0),
          resetForm = _c[0],
          setResetForm = _c[1];
        return react.createElement(
          react.Fragment,
          null,
          react.createElement(
            Button.A,
            {
              variant: 'contained',
              color: 'primary',
              size: 'small',
              onClick: function () {
                setResetForm(function (prev) {
                  return prev + 1;
                }),
                  setShowDialog(!0);
              },
            },
            'Request',
          ),
          react.createElement(
            LiftRequestDialog,
            lift_controls_assign(
              {
                key: resetForm,
                showFormDialog: showDialog,
                currentLevel: currentLevel || 'Unknown',
                availableDoorModes: requestDoorModes,
                availableRequestTypes: requestModes,
                onClose: function () {
                  for (var args = [], _i = 0; _i < arguments.length; _i++) args[_i] = arguments[_i];
                  setShowDialog(!1), onClose && onClose.apply(void 0, args);
                },
              },
              otherProps,
            ),
          ),
        );
      }
      LiftControls.__docgenInfo = { description: '', methods: [], displayName: 'LiftControls' };
      var LocalizationProvider = __webpack_require__(
          '../../node_modules/.pnpm/@mui+x-date-pickers@5.0.20_@emotion+react@11.9.3_@emotion+styled@11.9.3_@mui+material@5.8.7_@_kkp62yhptrthrgtgtakbrwpa6y/node_modules/@mui/x-date-pickers/LocalizationProvider/LocalizationProvider.js',
        ),
        AdapterDateFns = __webpack_require__(
          '../../node_modules/.pnpm/@mui+x-date-pickers@5.0.20_@emotion+react@11.9.3_@emotion+styled@11.9.3_@mui+material@5.8.7_@_kkp62yhptrthrgtgtakbrwpa6y/node_modules/@mui/x-date-pickers/AdapterDateFns/index.js',
        ),
        locale_awaiter = function (thisArg, _arguments, P, generator) {
          return new (P || (P = Promise))(function (resolve, reject) {
            function fulfilled(value) {
              try {
                step(generator.next(value));
              } catch (e) {
                reject(e);
              }
            }
            function rejected(value) {
              try {
                step(generator.throw(value));
              } catch (e) {
                reject(e);
              }
            }
            function step(result) {
              result.done
                ? resolve(result.value)
                : (function adopt(value) {
                    return value instanceof P
                      ? value
                      : new P(function (resolve) {
                          resolve(value);
                        });
                  })(result.value).then(fulfilled, rejected);
            }
            step((generator = generator.apply(thisArg, _arguments || [])).next());
          });
        },
        locale_generator = function (thisArg, body) {
          var f,
            y,
            t,
            g,
            _ = {
              label: 0,
              sent: function () {
                if (1 & t[0]) throw t[1];
                return t[1];
              },
              trys: [],
              ops: [],
            };
          return (
            (g = { next: verb(0), throw: verb(1), return: verb(2) }),
            'function' == typeof Symbol &&
              (g[Symbol.iterator] = function () {
                return this;
              }),
            g
          );
          function verb(n) {
            return function (v) {
              return (function step(op) {
                if (f) throw new TypeError('Generator is already executing.');
                for (; g && ((g = 0), op[0] && (_ = 0)), _; )
                  try {
                    if (
                      ((f = 1),
                      y &&
                        (t =
                          2 & op[0]
                            ? y.return
                            : op[0]
                              ? y.throw || ((t = y.return) && t.call(y), 0)
                              : y.next) &&
                        !(t = t.call(y, op[1])).done)
                    )
                      return t;
                    switch (((y = 0), t && (op = [2 & op[0], t.value]), op[0])) {
                      case 0:
                      case 1:
                        t = op;
                        break;
                      case 4:
                        return _.label++, { value: op[1], done: !1 };
                      case 5:
                        _.label++, (y = op[1]), (op = [0]);
                        continue;
                      case 7:
                        (op = _.ops.pop()), _.trys.pop();
                        continue;
                      default:
                        if (
                          !((t = _.trys),
                          (t = t.length > 0 && t[t.length - 1]) || (6 !== op[0] && 2 !== op[0]))
                        ) {
                          _ = 0;
                          continue;
                        }
                        if (3 === op[0] && (!t || (op[1] > t[0] && op[1] < t[3]))) {
                          _.label = op[1];
                          break;
                        }
                        if (6 === op[0] && _.label < t[1]) {
                          (_.label = t[1]), (t = op);
                          break;
                        }
                        if (t && _.label < t[2]) {
                          (_.label = t[2]), _.ops.push(op);
                          break;
                        }
                        t[2] && _.ops.pop(), _.trys.pop();
                        continue;
                    }
                    op = body.call(thisArg, _);
                  } catch (e) {
                    (op = [6, e]), (y = 0);
                  } finally {
                    f = t = 0;
                  }
                if (5 & op[0]) throw op[1];
                return { value: op[0] ? op[1] : void 0, done: !0 };
              })([n, v]);
            };
          }
        },
        locale_LocalizationProvider = function (_a) {
          var children = _a.children,
            _b = react.useState(null),
            locale = _b[0],
            setLocale = _b[1];
          return (
            react.useEffect(function () {
              locale_awaiter(void 0, void 0, void 0, function () {
                var _i, _a, lang, _b, _d;
                return locale_generator(this, function (_e) {
                  switch (_e.label) {
                    case 0:
                      (_i = 0), (_a = navigator.languages), (_e.label = 1);
                    case 1:
                      if (!(_i < _a.length)) return [3, 6];
                      (lang = _a[_i]), (_e.label = 2);
                    case 2:
                      return (
                        _e.trys.push([2, 4, , 5]),
                        (_b = setLocale),
                        [
                          4,
                          __webpack_require__(
                            '../react-components/node_modules/date-fns/locale ../../node_modules/.pnpm/node_modules/date-fns/locale lazy recursive ^\\.\\/.*\\/index\\.js$',
                          )('./'.concat(lang, '/index.js')),
                        ]
                      );
                    case 3:
                      return _b.apply(void 0, [_e.sent().default]), [2];
                    case 4:
                      return _e.sent(), [3, 5];
                    case 5:
                      return _i++, [3, 1];
                    case 6:
                      return (
                        (_d = setLocale),
                        [
                          4,
                          Promise.resolve().then(
                            __webpack_require__.bind(
                              __webpack_require__,
                              '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/esm/locale/en-US/index.js',
                            ),
                          ),
                        ]
                      );
                    case 7:
                      return _d.apply(void 0, [_e.sent().default]), [2];
                  }
                });
              });
            }, []),
            locale &&
              react.createElement(
                LocalizationProvider.$,
                { dateAdapter: AdapterDateFns.h, adapterLocale: locale },
                children,
              )
          );
        };
      locale_LocalizationProvider.__docgenInfo = {
        description: '',
        methods: [],
        displayName: 'LocalizationProvider',
      };
      var ButtonBase = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/ButtonBase/ButtonBase.js',
        ),
        logo_button_assign = function () {
          return (
            (logo_button_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            logo_button_assign.apply(this, arguments)
          );
        },
        logo_button_classes_logoBtn = 'logo-button-root',
        logo_button_classes_logoImg = 'logo-button-image',
        door_marker_assign =
          ((0, styled.Ay)(function (props) {
            return react.createElement(ButtonBase.A, logo_button_assign({}, props));
          })(function (_a) {
            var _b,
              theme = _a.theme;
            return (
              ((_b = {})['&.'.concat(logo_button_classes_logoBtn)] = {
                padding: ''.concat(theme.spacing(1), ' ').concat(theme.spacing(2)),
                boxSizing: 'border-box',
              }),
              (_b['& .'.concat(logo_button_classes_logoImg)] = { width: '100%', height: '100%' }),
              _b
            );
          }),
          function () {
            return (
              (door_marker_assign =
                Object.assign ||
                function (t) {
                  for (var s, i = 1, n = arguments.length; i < n; i++)
                    for (var p in (s = arguments[i]))
                      Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                  return t;
                }),
              door_marker_assign.apply(this, arguments)
            );
          }),
        door_marker_rest = function (s, e) {
          var t = {};
          for (var p in s)
            Object.prototype.hasOwnProperty.call(s, p) && e.indexOf(p) < 0 && (t[p] = s[p]);
          if (null != s && 'function' == typeof Object.getOwnPropertySymbols) {
            var i = 0;
            for (p = Object.getOwnPropertySymbols(s); i < p.length; i++)
              e.indexOf(p[i]) < 0 &&
                Object.prototype.propertyIsEnumerable.call(s, p[i]) &&
                (t[p[i]] = s[p[i]]);
          }
          return t;
        },
        door_marker_classes_marker = 'door-marker-base-marker',
        door_marker_classes_base = 'door-marker-base-door',
        door_marker_classes_open = 'door-marker-open',
        door_marker_classes_close = 'door-marker-close',
        door_marker_classes_moving = 'door-marker-moving',
        door_marker_classes_unknown = 'door-marker-unknown',
        door_marker_classes_transparent = 'door-marker-transparent',
        StyledG = (0, styled.Ay)('g')(function () {
          var _a;
          return (
            ((_a = {})['& .'.concat(door_marker_classes_marker)] = {
              cursor: 'pointer',
              pointerEvents: 'auto',
            }),
            (_a['& .'.concat(door_marker_classes_base)] = { strokeWidth: 0.2 }),
            (_a['& .'.concat(door_marker_classes_open)] = {
              width: 200,
              stroke: '#AFDDAE',
              strokeDasharray: 0.1,
            }),
            (_a['& .'.concat(door_marker_classes_close)] = { stroke: '#BC4812' }),
            (_a['& .'.concat(door_marker_classes_moving)] = {
              stroke: '#E9CE9F',
              strokeDasharray: 0.3,
            }),
            (_a['& .'.concat(door_marker_classes_unknown)] = { stroke: 'grey' }),
            (_a['& .'.concat(door_marker_classes_transparent)] = { stroke: 'transparent' }),
            _a
          );
        });
      function useDoorStyle(doorMode) {
        if (void 0 === doorMode) return door_marker_classes_unknown;
        switch (doorMode) {
          case dist.DoorMode.MODE_OPEN:
            return door_marker_classes_open;
          case dist.DoorMode.MODE_MOVING:
            return door_marker_classes_moving;
          case dist.DoorMode.MODE_CLOSED:
            return door_marker_classes_close;
          default:
            return door_marker_classes_unknown;
        }
      }
      var BaseDoor = function (_a) {
          var className = _a.className,
            otherProps = door_marker_rest(_a, ['className']);
          return react.createElement(
            'line',
            door_marker_assign(
              { className: (0, clsx_m.default)(door_marker_classes_base, className) },
              otherProps,
            ),
          );
        },
        DummyDoor = function (_a) {
          var className = _a.className,
            otherProps = door_marker_rest(_a, ['className']);
          return react.createElement(
            'line',
            door_marker_assign(
              {
                className: (0, clsx_m.default)(
                  door_marker_classes_base,
                  door_marker_classes_transparent,
                  className,
                ),
              },
              otherProps,
            ),
          );
        },
        SingleSwingDoor = function (_a) {
          var x1 = _a.x1,
            x2 = _a.x2,
            y1 = _a.y1,
            y2 = _a.y2,
            doorStyle = useDoorStyle(_a.doorMode);
          return react.createElement(
            react.Fragment,
            null,
            react.createElement(BaseDoor, { x1, y1, x2, y2, className: doorStyle }),
            react.createElement(DummyDoor, { x1, y1, x2, y2 }),
          );
        },
        SingleSlidingDoor = SingleSwingDoor,
        SingleTelescopeDoor = SingleSlidingDoor,
        DoubleSwingDoor = function (_a) {
          var x1 = _a.x1,
            y1 = _a.y1,
            x2 = _a.x2,
            y2 = _a.y2,
            separatorX = 0.002 * (x2 - x1),
            separatorY = 0.002 * (y2 - y1),
            centerX = x1 + (x2 - x1) / 2,
            centerY = y1 + (y2 - y1) / 2,
            doorStyle = useDoorStyle(_a.doorMode);
          return react.createElement(
            react.Fragment,
            null,
            react.createElement(BaseDoor, {
              x1,
              y1,
              x2: centerX - separatorX,
              y2: centerY - separatorY,
              className: doorStyle,
            }),
            react.createElement(BaseDoor, {
              x1: centerX + separatorX,
              y1: centerY + separatorY,
              x2,
              y2,
              className: doorStyle,
            }),
            react.createElement(DummyDoor, { x1, y1, x2, y2 }),
          );
        },
        DoubleSlidingDoor = DoubleSwingDoor,
        DoubleTelescopeDoor = DoubleSlidingDoor,
        DoorMarker = react.forwardRef(function (_a, ref) {
          var x1 = _a.x1,
            y1 = _a.y1,
            x2 = _a.x2,
            y2 = _a.y2,
            doorType = _a.doorType,
            doorMode = _a.doorMode,
            otherProps = door_marker_rest(_a, ['x1', 'y1', 'x2', 'y2', 'doorType', 'doorMode']),
            doorProps = { x1, y1, x2, y2, doorType, doorMode };
          try {
            return react.createElement(
              StyledG,
              door_marker_assign({ ref }, otherProps),
              react.createElement(
                'g',
                { className: otherProps.onClick ? door_marker_classes_marker : void 0 },
                (function () {
                  switch (doorType) {
                    case dist.Door.DOOR_TYPE_SINGLE_SWING:
                      return react.createElement(
                        SingleSwingDoor,
                        door_marker_assign({}, doorProps),
                      );
                    case dist.Door.DOOR_TYPE_SINGLE_SLIDING:
                      return react.createElement(
                        SingleSlidingDoor,
                        door_marker_assign({}, doorProps),
                      );
                    case dist.Door.DOOR_TYPE_SINGLE_TELESCOPE:
                      return react.createElement(
                        SingleTelescopeDoor,
                        door_marker_assign({}, doorProps),
                      );
                    case dist.Door.DOOR_TYPE_DOUBLE_SWING:
                      return react.createElement(
                        DoubleSwingDoor,
                        door_marker_assign({}, doorProps),
                      );
                    case dist.Door.DOOR_TYPE_DOUBLE_SLIDING:
                      return react.createElement(
                        DoubleSlidingDoor,
                        door_marker_assign({}, doorProps),
                      );
                    case dist.Door.DOOR_TYPE_DOUBLE_TELESCOPE:
                      return react.createElement(
                        DoubleTelescopeDoor,
                        door_marker_assign({}, doorProps),
                      );
                    default:
                      return null;
                  }
                })(),
              ),
            );
          } catch (e) {
            return console.error(e.message), null;
          }
        });
      DoorMarker.__docgenInfo = { description: '', methods: [], displayName: 'DoorMarker' };
      var extendStatics,
        rbush_min = __webpack_require__(
          '../../node_modules/.pnpm/rbush@3.0.1/node_modules/rbush/rbush.min.js',
        ),
        __extends =
          ((extendStatics = function (d, b) {
            return (
              (extendStatics =
                Object.setPrototypeOf ||
                ({ __proto__: [] } instanceof Array &&
                  function (d, b) {
                    d.__proto__ = b;
                  }) ||
                function (d, b) {
                  for (var p in b) Object.prototype.hasOwnProperty.call(b, p) && (d[p] = b[p]);
                }),
              extendStatics(d, b)
            );
          }),
          function (d, b) {
            if ('function' != typeof b && null !== b)
              throw new TypeError(
                'Class extends value ' + String(b) + ' is not a constructor or null',
              );
            function __() {
              this.constructor = d;
            }
            extendStatics(d, b),
              (d.prototype =
                null === b ? Object.create(b) : ((__.prototype = b.prototype), new __()));
          }),
        EntityRBush = (function (_super) {
          function EntityRBush() {
            return (null !== _super && _super.apply(this, arguments)) || this;
          }
          return (
            __extends(EntityRBush, _super),
            (EntityRBush.prototype.toBBox = function (entity) {
              return entity.bbox;
            }),
            EntityRBush
          );
        })(__webpack_require__.n(rbush_min)()),
        EntityManager = (function () {
          function EntityManager() {
            this._entities = new EntityRBush();
          }
          return (
            (EntityManager.prototype.add = function (entity) {
              return this._entities.insert(entity), entity;
            }),
            (EntityManager.prototype.remove = function (entity) {
              this._entities.remove(entity);
            }),
            (EntityManager.prototype.getNonColliding = function (bbox, _a) {
              for (
                var _b = void 0 === _a ? {} : _a,
                  _c = _b.searchDepth,
                  searchDepth = void 0 === _c ? 4 : _c,
                  _d = _b.distLimit,
                  distLimit = void 0 === _d ? 200 : _d,
                  distLimitSq = Math.pow(distLimit, 2),
                  candidates = [],
                  stack = [{ bbox, depth: 0 }],
                  _loop_1 = function (top_1) {
                    var search = top_1.bbox,
                      depth = top_1.depth,
                      collidingEntites = this_1._entities.search(search),
                      distSq =
                        Math.pow(search.minX - bbox.minX, 2) + Math.pow(search.minY - bbox.minY, 2);
                    0 === collidingEntites.length && distSq <= distLimitSq
                      ? candidates.push(search)
                      : depth < searchDepth &&
                        collidingEntites.forEach(function (colliding) {
                          stack.push({
                            bbox: {
                              minX: bbox.minX - (bbox.maxX - colliding.bbox.minX + 0.001),
                              minY: bbox.minY,
                              maxX: colliding.bbox.minX - 0.001,
                              maxY: bbox.maxY,
                            },
                            depth: depth + 1,
                          }),
                            stack.push({
                              bbox: {
                                minX: colliding.bbox.maxX + 0.001,
                                minY: bbox.minY,
                                maxX: bbox.maxX + (colliding.bbox.maxX - bbox.minX + 0.001),
                                maxY: bbox.maxY,
                              },
                              depth: depth + 1,
                            }),
                            stack.push({
                              bbox: {
                                minX: bbox.minX,
                                minY: bbox.minY - (bbox.maxY - colliding.bbox.minY + 0.001),
                                maxX: bbox.maxX,
                                maxY: colliding.bbox.minY - 0.001,
                              },
                              depth: depth + 1,
                            }),
                            stack.push({
                              bbox: {
                                minX: bbox.minX,
                                minY: colliding.bbox.maxY + 0.001,
                                maxX: bbox.maxX,
                                maxY: bbox.maxY + (colliding.bbox.maxY - bbox.minY + 0.001),
                              },
                              depth: depth + 1,
                            });
                        });
                  },
                  this_1 = this,
                  top_1 = stack.pop();
                void 0 !== top_1;
                top_1 = stack.pop()
              )
                _loop_1(top_1);
              if (0 === candidates.length) return null;
              var preferredCenterX = bbox.minX + (bbox.maxX - bbox.minX) / 2,
                preferredCenterY = bbox.minY + (bbox.maxY - bbox.minY) / 2,
                distances = candidates.map(function (candidate) {
                  var centerX = candidate.minX + (candidate.maxX - candidate.minX) / 2,
                    centerY = candidate.minY + (candidate.maxY - candidate.minY) / 2;
                  return (
                    Math.pow(preferredCenterY - centerY, 2) +
                    Math.pow(preferredCenterX - centerX, 2)
                  );
                }),
                closestIdx = 0,
                closestDistance = distances[0];
              return (
                distances.forEach(function (d, i) {
                  d < closestDistance && ((closestDistance = d), (closestIdx = i));
                }),
                candidates[closestIdx]
              );
            }),
            (EntityManager.prototype.collides = function (bbox) {
              return this._entities.collides(bbox);
            }),
            EntityManager
          );
        })(),
        EntityManagerContext = react.createContext(new EntityManager()),
        context = __webpack_require__(
          '../../node_modules/.pnpm/react-leaflet@2.8.0_leaflet@1.8.0_react-dom@18.2.0_react@18.2.0/node_modules/react-leaflet/es/context.js',
        ),
        DefaultMarkerActualSizeMinZoom = 6;
      var lift_marker_assign = function () {
          return (
            (lift_marker_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            lift_marker_assign.apply(this, arguments)
          );
        },
        lift_marker_rest = function (s, e) {
          var t = {};
          for (var p in s)
            Object.prototype.hasOwnProperty.call(s, p) && e.indexOf(p) < 0 && (t[p] = s[p]);
          if (null != s && 'function' == typeof Object.getOwnPropertySymbols) {
            var i = 0;
            for (p = Object.getOwnPropertySymbols(s); i < p.length; i++)
              e.indexOf(p[i]) < 0 &&
                Object.prototype.propertyIsEnumerable.call(s, p[i]) &&
                (t[p[i]] = s[p[i]]);
          }
          return t;
        },
        getLiftModeText = function (liftState) {
          if (!liftState.current_mode) return 'UNKNOWN';
          switch (liftState.current_mode) {
            case dist.LiftState.MODE_FIRE:
              return 'FIRE!';
            case dist.LiftState.MODE_EMERGENCY:
              return 'EMERGENCY!';
            case dist.LiftState.MODE_OFFLINE:
              return 'OFFLINE';
            default:
              return 'NORMAL';
          }
        };
      var liftMarkerClasses = {
          marker: 'lift-marker-root',
          lift: 'lift-marker-lift',
          text: 'lift-marker-text',
          onCurrentLevel: 'lift-marker-oncurrentlevel',
          moving: 'lift-marker-moving',
          unknown: 'lift-marker-unknown',
          emergency: 'lift-marker-emergency',
          fire: 'lift-marker-fire',
          offLine: 'lift-marker-offline',
          human: 'lift-marker-human',
        },
        lift_marker_StyledG = (0, styled.Ay)('g')(function (_a) {
          var _b,
            theme = _a.theme;
          return (
            ((_b = {})['&.'.concat(liftMarkerClasses.marker)] = {
              cursor: 'pointer',
              pointerEvents: 'auto',
            }),
            (_b['& .'.concat(liftMarkerClasses.lift)] = { strokeWidth: '0.2' }),
            (_b['& .'.concat(liftMarkerClasses.text)] = {
              dominantBaseline: 'central',
              textAnchor: 'middle',
              fontSize: '0.16px',
              fontWeight: 'bold',
              cursor: 'inherit',
              userSelect: 'none',
            }),
            (_b['& .'.concat(liftMarkerClasses.onCurrentLevel)] = {
              fill: theme.palette.success.light,
              opacity: '70%',
            }),
            (_b['& .'.concat(liftMarkerClasses.moving)] = {
              fill: theme.palette.secondary.light,
              opacity: '70%',
            }),
            (_b['& .'.concat(liftMarkerClasses.unknown)] = {
              fill: theme.palette.warning.light,
              opacity: '80%',
            }),
            (_b['& .'.concat(liftMarkerClasses.emergency)] = {
              fill: theme.palette.error.light,
              opacity: '80%',
            }),
            (_b['& .'.concat(liftMarkerClasses.fire)] = {
              fill: theme.palette.error.main,
              opacity: '80%',
            }),
            (_b['& .'.concat(liftMarkerClasses.offLine)] = {
              fill: theme.palette.grey[400],
              opacity: '80%',
            }),
            (_b['& .'.concat(liftMarkerClasses.human)] = {
              fill: theme.palette.info.main,
              opacity: '80%',
            }),
            _b
          );
        }),
        LiftMarker = react.forwardRef(function (_a, ref) {
          var textScale,
            cx = _a.cx,
            cy = _a.cy,
            width = _a.width,
            height = _a.height,
            yaw = _a.yaw,
            liftState = _a.liftState,
            variant = _a.variant,
            otherProps = lift_marker_rest(_a, [
              'cx',
              'cy',
              'width',
              'height',
              'yaw',
              'liftState',
              'variant',
            ]),
            markerClass = variant ? liftMarkerClasses[variant] : liftMarkerClasses.onCurrentLevel,
            x = cx - width / 2,
            y = cy - height / 2,
            r = 0.04 * Math.max(width, height);
          return react.createElement(
            lift_marker_StyledG,
            lift_marker_assign(
              {
                ref,
                className: (0, clsx_m.default)(
                  otherProps.onClick ? liftMarkerClasses.marker : void 0,
                  otherProps.className,
                ),
              },
              otherProps,
            ),
            react.createElement('rect', {
              className: ''.concat(liftMarkerClasses.lift, ' ').concat(markerClass),
              x,
              y,
              width,
              height,
              rx: r,
              ry: r,
              style: {
                transform: 'rotate('.concat(yaw, 'deg)'),
                transformOrigin: ''.concat(cx, 'px ').concat(cy, 'px'),
              },
            }),
            ((textScale = Math.min(width, height)),
            liftState
              ? react.createElement(
                  'text',
                  {
                    className: liftMarkerClasses.text,
                    transform: 'translate('
                      .concat(cx, ' ')
                      .concat(cy, ') scale(')
                      .concat(textScale, ')'),
                  },
                  react.createElement('tspan', { x: '0', dy: '-1.8em' }, liftState.current_floor),
                  react.createElement(
                    'tspan',
                    { x: '0', dy: '1.2em', fontSize: '0.7em' },
                    getLiftModeText(liftState),
                  ),
                  react.createElement(
                    'tspan',
                    { x: '0', dy: '0.6em', fontSize: '3em' },
                    (function getLiftMotionText(liftState) {
                      switch (liftState.motion_state) {
                        case dist.LiftState.MOTION_UP:
                          return '▲';
                        case dist.LiftState.MOTION_DOWN:
                          return '▼';
                        case dist.LiftState.MOTION_STOPPED:
                          return '⯀';
                        default:
                          return '?';
                      }
                    })(liftState),
                  ),
                )
              : react.createElement(
                  'text',
                  {
                    className: liftMarkerClasses.text,
                    transform: 'translate('
                      .concat(cx, ' ')
                      .concat(cy, ') scale(')
                      .concat(textScale, ')'),
                  },
                  react.createElement('tspan', { x: '0', dy: '-0.5em' }, 'Unknown'),
                  react.createElement('tspan', { x: '0', dy: '1em' }, 'State'),
                )),
          );
        });
      LiftMarker.__docgenInfo = { description: '', methods: [], displayName: 'LiftMarker' };
      var leaflet_src = __webpack_require__(
          '../../node_modules/.pnpm/leaflet@1.8.0/node_modules/leaflet/dist/leaflet-src.js',
        ),
        leaflet_src_default = __webpack_require__.n(leaflet_src),
        Map =
          (__webpack_require__(
            '../../node_modules/.pnpm/leaflet@1.8.0/node_modules/leaflet/dist/leaflet.css',
          ),
          __webpack_require__(
            '../../node_modules/.pnpm/react-leaflet@2.8.0_leaflet@1.8.0_react-dom@18.2.0_react@18.2.0/node_modules/react-leaflet/es/Map.js',
          )),
        Pane = __webpack_require__(
          '../../node_modules/.pnpm/react-leaflet@2.8.0_leaflet@1.8.0_react-dom@18.2.0_react@18.2.0/node_modules/react-leaflet/es/Pane.js',
        ),
        labels_overlay_LabelsPortalContext = react.createContext(null),
        svg_overlay_SVGOverlay = __webpack_require__(
          '../../node_modules/.pnpm/react-leaflet@2.8.0_leaflet@1.8.0_react-dom@18.2.0_react@18.2.0/node_modules/react-leaflet/es/SVGOverlay.js',
        ).A;
      var three_module = __webpack_require__(
        '../../node_modules/.pnpm/three@0.156.1/node_modules/three/build/three.module.js',
      );
      var map_assign = function () {
          return (
            (map_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            map_assign.apply(this, arguments)
          );
        },
        map_rest = function (s, e) {
          var t = {};
          for (var p in s)
            Object.prototype.hasOwnProperty.call(s, p) && e.indexOf(p) < 0 && (t[p] = s[p]);
          if (null != s && 'function' == typeof Object.getOwnPropertySymbols) {
            var i = 0;
            for (p = Object.getOwnPropertySymbols(s); i < p.length; i++)
              e.indexOf(p[i]) < 0 &&
                Object.prototype.propertyIsEnumerable.call(s, p[i]) &&
                (t[p[i]] = s[p[i]]);
          }
          return t;
        };
      function EntityManagerProvider(_a) {
        var children = _a.children,
          leaflet = (0, context.jF)(),
          entityManager = react.useRef(new EntityManager()).current;
        return (
          react.useEffect(
            function () {
              if (leaflet.map) {
                var listener = function () {};
                return (
                  leaflet.map.on('zoom', listener),
                  function () {
                    leaflet.map && leaflet.map.off('zoom', listener);
                  }
                );
              }
            },
            [leaflet, leaflet.map],
          ),
          entityManager
            ? react.createElement(EntityManagerContext.Provider, { value: entityManager }, children)
            : null
        );
      }
      var LMap = react.forwardRef(function (_a, ref) {
        var children = _a.children,
          bounds = _a.bounds,
          otherProps = map_rest(_a, ['children', 'bounds']),
          _b = react.useState(null),
          labelsPortal = _b[0],
          setLabelsPortal = _b[1],
          viewBox = bounds
            ? (function viewBoxFromLeafletBounds(bounds) {
                var lbounds =
                    bounds instanceof leaflet_src_default().LatLngBounds
                      ? bounds
                      : new (leaflet_src_default().LatLngBounds)(bounds),
                  width = lbounds.getEast() - lbounds.getWest(),
                  height = lbounds.getNorth() - lbounds.getSouth();
                return ''
                  .concat(lbounds.getWest(), ' ')
                  .concat(lbounds.getNorth(), ' ')
                  .concat(width, ' ')
                  .concat(height);
              })(bounds)
            : void 0,
          theme = (0, useTheme.A)();
        return react.createElement(
          Map.A,
          map_assign(
            {
              ref,
              style: {
                height: '100%',
                width: '100%',
                margin: 0,
                padding: 0,
                backgroundColor: theme.palette.background.default,
              },
              crs: leaflet_src.CRS.Simple,
            },
            otherProps,
          ),
          react.createElement(
            EntityManagerProvider,
            null,
            react.createElement(
              labels_overlay_LabelsPortalContext.Provider,
              { value: labelsPortal },
              children,
              bounds &&
                react.createElement(
                  Pane.A,
                  { name: 'label', style: { zIndex: 1e3 } },
                  react.createElement(svg_overlay_SVGOverlay, {
                    ref: function (current) {
                      setLabelsPortal((null == current ? void 0 : current.container) || null);
                    },
                    viewBox,
                    bounds,
                  }),
                ),
            ),
          ),
        );
      });
      LMap.__docgenInfo = { description: '', methods: [], displayName: 'LMap' };
      var src_browser = __webpack_require__(
          '../../node_modules/.pnpm/debug@4.3.4_supports-color@5.5.0/node_modules/debug/src/browser.js',
        ),
        src_browser_default = __webpack_require__.n(src_browser),
        default_robot_marker_assign = function () {
          return (
            (default_robot_marker_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            default_robot_marker_assign.apply(this, arguments)
          );
        };
      function makeGradientShadow(color) {
        return function (props) {
          return react.createElement(
            'radialGradient',
            default_robot_marker_assign({}, props),
            react.createElement('stop', { offset: '70%', stopColor: ''.concat(color, 'ff') }),
            react.createElement('stop', { offset: '75%', stopColor: ''.concat(color, '80') }),
            react.createElement('stop', { offset: '80%', stopColor: ''.concat(color, '60') }),
            react.createElement('stop', { offset: '85%', stopColor: ''.concat(color, '30') }),
            react.createElement('stop', { offset: '90%', stopColor: ''.concat(color, '18') }),
            react.createElement('stop', { offset: '95%', stopColor: ''.concat(color, '08') }),
            react.createElement('stop', { offset: '100%', stopColor: ''.concat(color, '00') }),
          );
        };
      }
      var DefaultMarker = function (_a) {
        var cx = _a.cx,
          cy = _a.cy,
          r = _a.r,
          color = _a.color,
          _b = _a.inConflict,
          inConflict = void 0 !== _b && _b,
          colorManager = react.useContext(ColorContext),
          theme = (0, useTheme.A)(),
          componentId = react.useMemo(uniqueId, []),
          shadowId = react.useMemo(
            function () {
              return 'RobotDefaultIcon-'.concat(componentId, '-shadow');
            },
            [componentId],
          ),
          conflictShadowId = react.useMemo(
            function () {
              return 'RobotDefaultIcon-'.concat(componentId, '-shadow-conflict');
            },
            [componentId],
          ),
          Shadow = react.useMemo(function () {
            return makeGradientShadow('#000000');
          }, []),
          ShadowConflict = react.useMemo(
            function () {
              return makeGradientShadow(colorManager.conflictHighlight);
            },
            [colorManager.conflictHighlight],
          );
        return react.createElement(
          'g',
          null,
          react.createElement(
            'defs',
            null,
            react.createElement(Shadow, { id: shadowId }),
            react.createElement(ShadowConflict, { id: conflictShadowId }),
          ),
          react.createElement('circle', {
            r: 1.3 * r,
            cx,
            cy,
            fill: 'url(#'.concat(inConflict ? conflictShadowId : shadowId, ')'),
          }),
          react.createElement('circle', { r, cx, cy, fill: color }),
          react.createElement('line', {
            x1: cx,
            y1: cy,
            x2: cx + r,
            y2: cy,
            stroke: theme.palette.common.black,
            strokeWidth: '0.05',
          }),
        );
      };
      DefaultMarker.__docgenInfo = { description: '', methods: [], displayName: 'DefaultMarker' };
      var image_marker_assign = function () {
        return (
          (image_marker_assign =
            Object.assign ||
            function (t) {
              for (var s, i = 1, n = arguments.length; i < n; i++)
                for (var p in (s = arguments[i]))
                  Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
              return t;
            }),
          image_marker_assign.apply(this, arguments)
        );
      };
      function image_marker_makeGradientShadow(color) {
        return function (props) {
          return react.createElement(
            'radialGradient',
            image_marker_assign({}, props),
            react.createElement('stop', { offset: '0%', stopColor: ''.concat(color, '80') }),
            react.createElement('stop', { offset: '70%', stopColor: ''.concat(color, '40') }),
            react.createElement('stop', { offset: '90%', stopColor: ''.concat(color, '10') }),
            react.createElement('stop', { offset: '100%', stopColor: ''.concat(color, '00') }),
          );
        };
      }
      var ImageMarker = function (_a) {
        var cx = _a.cx,
          cy = _a.cy,
          r = _a.r,
          iconPath = _a.iconPath,
          _b = _a.inConflict,
          inConflict = void 0 !== _b && _b,
          onError = _a.onError,
          colorManager = react.useContext(ColorContext),
          componentId = react.useMemo(uniqueId, []),
          shadowId = react.useMemo(
            function () {
              return 'RobotImageIcon-'.concat(componentId, '-shadow');
            },
            [componentId],
          ),
          conflictShadowId = react.useMemo(
            function () {
              return 'RobotImageIcon-'.concat(componentId, '-shadow-conflict');
            },
            [componentId],
          ),
          Shadow = react.useMemo(function () {
            return image_marker_makeGradientShadow('#000000');
          }, []),
          ShadowConflict = react.useMemo(
            function () {
              return image_marker_makeGradientShadow(colorManager.conflictHighlight);
            },
            [colorManager.conflictHighlight],
          );
        return iconPath
          ? react.createElement(
              'g',
              null,
              react.createElement(
                'defs',
                null,
                react.createElement(Shadow, { id: shadowId }),
                react.createElement(ShadowConflict, { id: conflictShadowId }),
              ),
              react.createElement('circle', {
                r: 1.3 * r,
                cx,
                cy,
                fill: 'url(#'.concat(inConflict ? conflictShadowId : shadowId, ')'),
              }),
              react.createElement('image', {
                href: iconPath,
                width: 2 * r,
                height: 2 * r,
                x: cx - r,
                y: cy - r,
                onError,
              }),
            )
          : null;
      };
      ImageMarker.__docgenInfo = {
        description: 'Image should be 1x1 aspect ratio.',
        methods: [],
        displayName: 'ImageMarker',
      };
      var robot_marker_assign = function () {
          return (
            (robot_marker_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            robot_marker_assign.apply(this, arguments)
          );
        },
        robot_marker_rest = function (s, e) {
          var t = {};
          for (var p in s)
            Object.prototype.hasOwnProperty.call(s, p) && e.indexOf(p) < 0 && (t[p] = s[p]);
          if (null != s && 'function' == typeof Object.getOwnPropertySymbols) {
            var i = 0;
            for (p = Object.getOwnPropertySymbols(s); i < p.length; i++)
              e.indexOf(p[i]) < 0 &&
                Object.prototype.propertyIsEnumerable.call(s, p[i]) &&
                (t[p[i]] = s[p[i]]);
          }
          return t;
        },
        debug = src_browser_default()('Map:RobotMarker'),
        robot_marker_classes_clickable = 'robot-marker-clickable',
        robot_marker_StyledG = (0, styled.Ay)('g')(function () {
          var _a;
          return (
            ((_a = {})['& .'.concat(robot_marker_classes_clickable)] = {
              pointerEvents: 'auto',
              cursor: 'pointer',
            }),
            _a
          );
        }),
        RobotMarker = react.forwardRef(function (_a, ref) {
          var cx = _a.cx,
            cy = _a.cy,
            r = _a.r,
            color = _a.color,
            inConflict = _a.inConflict,
            iconPath = _a.iconPath,
            otherProps = robot_marker_rest(_a, [
              'cx',
              'cy',
              'r',
              'color',
              'inConflict',
              'iconPath',
            ]);
          debug('render');
          var _b = react.useState(!1),
            imageHasError = _b[0],
            setImageHasError = _b[1],
            useImageMarker = !!iconPath && !imageHasError,
            imageErrorHandler = react.useCallback(function () {
              return setImageHasError(!0);
            }, []);
          return react.createElement(
            robot_marker_StyledG,
            robot_marker_assign({ ref }, otherProps),
            react.createElement(
              'g',
              {
                className: (0, clsx_m.default)(
                  otherProps.onClick && robot_marker_classes_clickable,
                ),
              },
              useImageMarker
                ? react.createElement(ImageMarker, {
                    cx,
                    cy,
                    r,
                    iconPath,
                    onError: imageErrorHandler,
                    inConflict,
                  })
                : react.createElement(DefaultMarker, { cx, cy, r, color, inConflict }),
            ),
          );
        });
      RobotMarker.__docgenInfo = { description: '', methods: [], displayName: 'RobotMarker' };
      function segmentToCoefficientSet(segment) {
        var x0 = segment.initialPose,
          x1 = segment.finalPose,
          v0 = segment.initialVelocity,
          v1 = segment.finalVelocity,
          initialTime = segment.initialTime,
          dt = segment.finalTime - initialTime,
          w0 = v0 / dt,
          w1 = v1 / dt;
        return { a: w1 + w0 - 2 * x1 + 2 * x0, b: -w1 - 2 * w0 + 3 * x1 - 3 * x0, c: w0, d: x0 };
      }
      function assignKnotsToSegment(knot, nextKnot, forCoordinate) {
        return {
          initialPose: knot.pose[forCoordinate],
          finalPose: nextKnot.pose[forCoordinate],
          initialVelocity: knot.velocity[forCoordinate],
          finalVelocity: nextKnot.velocity[forCoordinate],
          initialTime: knot.time,
          finalTime: nextKnot.time,
        };
      }
      function bezierHelper(coeffs) {
        var a = coeffs.a,
          b = coeffs.b,
          c = coeffs.c,
          d = coeffs.d,
          p1 = (c + 3 * d) / 3,
          p2 = (b - 3 * d + 6 * p1) / 3;
        return [d, p1, p2, a + d - 3 * p1 + 3 * p2];
      }
      function bezierControlPoints(segmentCoefficients) {
        var px = bezierHelper(segmentCoefficients.x),
          py = bezierHelper(segmentCoefficients.y);
        return px.map(function (x, i) {
          return [x, py[i]];
        });
      }
      function trajectoryPath(trajectorySegments) {
        var knots = (function rawKnotsToKnots(rawKnots) {
            for (var knots = [], _i = 0, rawKnots_1 = rawKnots; _i < rawKnots_1.length; _i++) {
              var rawKnot = rawKnots_1[_i],
                _a = rawKnot.x,
                poseX = _a[0],
                poseY = _a[1],
                poseTheta = _a[2],
                _b = rawKnot.v,
                velocityX = _b[0],
                velocityY = _b[1],
                velocityTheta = _b[2];
              knots.push({
                pose: { x: poseX, y: poseY, theta: poseTheta },
                velocity: { x: velocityX, y: velocityY, theta: velocityTheta },
                time: rawKnot.t,
              });
            }
            return knots;
          })(trajectorySegments),
          coeff = (function knotsToSegmentCoefficientsArray(knots) {
            for (var scs = [], i = 0; i < knots.length - 1; ++i) {
              var knot = knots[i],
                nextKnot = knots[i + 1],
                sc = {
                  x: segmentToCoefficientSet(assignKnotsToSegment(knot, nextKnot, 'x')),
                  y: segmentToCoefficientSet(assignKnotsToSegment(knot, nextKnot, 'y')),
                  theta: segmentToCoefficientSet(assignKnotsToSegment(knot, nextKnot, 'theta')),
                  initialTime: knot.time,
                  finalTime: nextKnot.time,
                };
              scs.push(sc);
            }
            return scs;
          })(knots),
          bezierSplines = coeff.map(bezierControlPoints),
          totalDuration = knots[knots.length - 1].time - knots[0].time,
          segOffsets = knots.map(function (k) {
            return (k.time - knots[0].time) / totalDuration;
          }),
          d = 'M '.concat(bezierSplines[0][0][0], ' ').concat(-bezierSplines[0][0][1], ' C ');
        return (
          bezierSplines.map(function (bzCurves) {
            return (d +=
              ''.concat(bzCurves[1][0], ' ').concat(-bzCurves[1][1], ' ') +
              ''.concat(bzCurves[2][0], ' ').concat(-bzCurves[2][1], ' ') +
              ''.concat(bzCurves[3][0], ' ').concat(-bzCurves[3][1], ' '));
          }),
          { d, segOffsets }
        );
      }
      var ConflictPath = function (props) {
          var d = props.d,
            trajectory = props.trajectory,
            footprint = props.footprint,
            theme = (0, useTheme.A)();
          return react.createElement(
            react.Fragment,
            null,
            react.createElement(
              'mask',
              { id: 'mask-'.concat(trajectory.id), maskUnits: 'userSpaceOnUse' },
              react.createElement('path', {
                d,
                stroke: 'white',
                strokeWidth: footprint,
                strokeLinecap: 'round',
                fill: 'none',
              }),
              react.createElement('path', {
                d,
                stroke: 'black',
                strokeWidth: 0.8 * footprint,
                strokeLinecap: 'round',
                fill: 'none',
              }),
            ),
            react.createElement('path', {
              d,
              stroke: theme.palette.error.main,
              strokeWidth: footprint,
              strokeLinecap: 'round',
              fill: 'none',
              mask: 'url(#mask-'.concat(trajectory.id, ')'),
            }),
          );
        },
        FollowAnimationPath = function (props) {
          var trajectory = props.trajectory,
            d = props.d,
            color = props.color,
            conflict = props.conflict,
            footprint = props.footprint,
            _a = props.animationScale,
            animationScale = void 0 === _a ? 1 : _a,
            _b = props.loopAnimation,
            loopAnimation = void 0 !== _b && _b,
            pathRef = react.useRef(null);
          return (
            react.useLayoutEffect(
              function () {
                if (pathRef.current) {
                  var offsets = (function keyframeOffsets(traj) {
                      var segments = traj.segments,
                        totalDuration = segments[segments.length - 1].t - segments[0].t;
                      return traj.segments.map(function (seg) {
                        return (seg.t - segments[0].t) / totalDuration;
                      });
                    })(trajectory),
                    pathAnim = pathRef.current,
                    totalLength = pathAnim.getTotalLength();
                  if (!(totalLength < 0.01)) {
                    var traj,
                      scale,
                      strokeDash = Number(pathAnim.getAttribute('stroke-width') || 1) / totalLength;
                    return (
                      pathAnim.setAttribute(
                        'stroke-dasharray',
                        ''.concat(strokeDash, ' ').concat(2 - strokeDash),
                      ),
                      pathAnim.animate(
                        offsets.map(function (offset) {
                          return { offset, strokeDashoffset: Math.max(2 - offset, strokeDash + 1) };
                        }),
                        {
                          duration:
                            ((traj = trajectory),
                            (scale = animationScale),
                            (traj.segments[traj.segments.length - 1].t - traj.segments[0].t) /
                              scale),
                          easing: 'linear',
                          fill: 'forwards',
                          iterations: loopAnimation ? 1 / 0 : 1,
                        },
                      ),
                      function () {
                        pathAnim.getAnimations().forEach(function (anim) {
                          return anim.cancel();
                        });
                      }
                    );
                  }
                }
              },
              [animationScale, loopAnimation, trajectory],
            ),
            react.createElement(
              react.Fragment,
              null,
              react.createElement('path', {
                d,
                stroke: color,
                opacity: 0.4,
                strokeWidth: conflict ? 0.8 * footprint : footprint,
                strokeLinecap: 'round',
                fill: 'none',
              }),
              react.createElement('path', {
                ref: pathRef,
                d,
                stroke: color,
                opacity: 0.8,
                strokeWidth: conflict ? 0.8 * footprint : footprint,
                strokeLinecap: 'round',
                fill: 'none',
                pathLength: 1,
                strokeDasharray: 2,
                strokeDashoffset: 2,
              }),
              conflict ? react.createElement(ConflictPath, { d, trajectory, footprint }) : null,
            )
          );
        };
      FollowAnimationPath.__docgenInfo = {
        description: '',
        methods: [],
        displayName: 'FollowAnimationPath',
      };
      var trajectory_marker_assign = function () {
          return (
            (trajectory_marker_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            trajectory_marker_assign.apply(this, arguments)
          );
        },
        trajectory_marker_rest = function (s, e) {
          var t = {};
          for (var p in s)
            Object.prototype.hasOwnProperty.call(s, p) && e.indexOf(p) < 0 && (t[p] = s[p]);
          if (null != s && 'function' == typeof Object.getOwnPropertySymbols) {
            var i = 0;
            for (p = Object.getOwnPropertySymbols(s); i < p.length; i++)
              e.indexOf(p[i]) < 0 &&
                Object.prototype.propertyIsEnumerable.call(s, p[i]) &&
                (t[p[i]] = s[p[i]]);
          }
          return t;
        },
        trajectory_marker_debug = src_browser_default()('Map:TrajectoryMarker'),
        TrajectoryMarker = react.forwardRef(function (_a, ref) {
          var trajectory = _a.trajectory,
            color = _a.color,
            _b = _a.conflict,
            conflict = void 0 !== _b && _b,
            _c = _a.loopAnimation,
            loopAnimation = void 0 !== _c && _c,
            _d = _a.animationScale,
            animationScale = void 0 === _d ? 1 : _d,
            otherProps = trajectory_marker_rest(_a, [
              'trajectory',
              'color',
              'conflict',
              'loopAnimation',
              'animationScale',
            ]);
          trajectory_marker_debug('render '.concat(trajectory.id));
          var footprint = trajectory.dimensions,
            pathD = react.useMemo(
              function () {
                return trajectoryPath(trajectory.segments).d;
              },
              [trajectory],
            );
          return react.createElement(
            'g',
            trajectory_marker_assign({ ref }, otherProps),
            react.createElement(FollowAnimationPath, {
              trajectory,
              d: pathD,
              color,
              footprint,
              conflict,
              loopAnimation,
              animationScale,
            }),
          );
        });
      TrajectoryMarker.__docgenInfo = {
        description: '',
        methods: [],
        displayName: 'TrajectoryMarker',
      };
      __webpack_require__(
        '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Slider/Slider.js',
      ),
        __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/Gesture.js',
        ),
        __webpack_require__(
          '../../node_modules/.pnpm/react-dom@18.2.0_react@18.2.0/node_modules/react-dom/index.js',
        );
      var MapControl = __webpack_require__(
          '../../node_modules/.pnpm/react-leaflet@2.8.0_leaflet@1.8.0_react-dom@18.2.0_react@18.2.0/node_modules/react-leaflet/es/MapControl.js',
        ),
        trajectory_time_control_extends = (function () {
          var extendStatics = function (d, b) {
            return (
              (extendStatics =
                Object.setPrototypeOf ||
                ({ __proto__: [] } instanceof Array &&
                  function (d, b) {
                    d.__proto__ = b;
                  }) ||
                function (d, b) {
                  for (var p in b) Object.prototype.hasOwnProperty.call(b, p) && (d[p] = b[p]);
                }),
              extendStatics(d, b)
            );
          };
          return function (d, b) {
            if ('function' != typeof b && null !== b)
              throw new TypeError(
                'Class extends value ' + String(b) + ' is not a constructor or null',
              );
            function __() {
              this.constructor = d;
            }
            extendStatics(d, b),
              (d.prototype =
                null === b ? Object.create(b) : ((__.prototype = b.prototype), new __()));
          };
        })(),
        trajectory_time_control_assign = function () {
          return (
            (trajectory_time_control_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            trajectory_time_control_assign.apply(this, arguments)
          );
        },
        trajectory_time_control_classes_root = 'traj-time-control-root',
        trajectory_time_control_classes_container = 'traj-time-control-container',
        trajectory_time_control_classes_slider = 'traj-time-control-slider',
        trajectory_time_control_classes_textField = 'traj-time-control-textfield';
      (0, styled.Ay)('div')(function () {
        var _a;
        return (
          ((_a = {})['&.'.concat(trajectory_time_control_classes_root)] = {
            width: '100%',
            height: '100%',
          }),
          (_a['& .'.concat(trajectory_time_control_classes_container)] = { padding: 16 }),
          (_a['& .'.concat(trajectory_time_control_classes_slider)] = {
            width: 200,
            verticalAlign: 'middle',
          }),
          (_a['& .'.concat(trajectory_time_control_classes_textField)] = { width: '3em' }),
          _a
        );
      });
      var BaseTrajectoryTimeControl = (function (_super) {
        function BaseTrajectoryTimeControl(props) {
          var _this = _super.call(this, props) || this;
          return (
            (_this._container = leaflet_src_default().DomUtil.create('div')),
            (_this._container.className = 'leaflet-control-layers'),
            _this
          );
        }
        return (
          trajectory_time_control_extends(BaseTrajectoryTimeControl, _super),
          (BaseTrajectoryTimeControl.prototype.createLeafletElement = function (props) {
            var _this = this;
            return new (leaflet_src_default().Control.extend({
              onAdd: function () {
                return (
                  trajectory_time_control_assign({}, props), _this._container, _this._container
                );
              },
            }))(props);
          }),
          (BaseTrajectoryTimeControl.prototype.updateLeafletElement = function (
            _fromProps,
            toProps,
          ) {
            trajectory_time_control_assign({}, toProps), this._container;
          }),
          BaseTrajectoryTimeControl
        );
      })(MapControl.A);
      (0, context.k0)(BaseTrajectoryTimeControl);
      var waypoint_marker_assign = function () {
          return (
            (waypoint_marker_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            waypoint_marker_assign.apply(this, arguments)
          );
        },
        waypoint_marker_rest = function (s, e) {
          var t = {};
          for (var p in s)
            Object.prototype.hasOwnProperty.call(s, p) && e.indexOf(p) < 0 && (t[p] = s[p]);
          if (null != s && 'function' == typeof Object.getOwnPropertySymbols) {
            var i = 0;
            for (p = Object.getOwnPropertySymbols(s); i < p.length; i++)
              e.indexOf(p[i]) < 0 &&
                Object.prototype.propertyIsEnumerable.call(s, p[i]) &&
                (t[p[i]] = s[p[i]]);
          }
          return t;
        },
        waypoint_marker_debug = src_browser_default()('Map:WaypointMarker'),
        waypoint_marker_classes_marker = 'waypoint-marker-marker',
        waypoint_marker_StyledG = (0, styled.Ay)('g')(function () {
          var _a;
          return (
            ((_a = {})['& .'.concat(waypoint_marker_classes_marker)] = { pointerEvents: 'none' }),
            _a
          );
        }),
        WaypointMarker = react.forwardRef(function (_a, ref) {
          var cx = _a.cx,
            cy = _a.cy,
            size = _a.size,
            otherProps = waypoint_marker_rest(_a, ['cx', 'cy', 'size']);
          waypoint_marker_debug('render');
          var waypointId = react.useMemo(uniqueId, []);
          return react.createElement(
            waypoint_marker_StyledG,
            waypoint_marker_assign({ ref }, otherProps),
            react.createElement(
              'defs',
              null,
              react.createElement(
                'filter',
                {
                  id: 'waypoint-'.concat(waypointId, '-shadow'),
                  x: '-20%',
                  y: '-20%',
                  width: '140%',
                  height: '140%',
                  filterUnits: 'userSpaceOnUse',
                },
                react.createElement('feDropShadow', {
                  dx: -0.05 * size,
                  dy: -0.05 * size,
                  stdDeviation: 0.15 * size,
                  floodColor: 'black',
                }),
              ),
            ),
            react.createElement('rect', {
              className: waypoint_marker_classes_marker,
              x: cx - size / 2,
              y: cy - size / 2,
              width: size,
              height: size,
              fill: '#FFBF00',
              filter: 'url(#waypoint-'.concat(waypointId, '-shadow)'),
            }),
          );
        });
      WaypointMarker.__docgenInfo = { description: '', methods: [], displayName: 'WaypointMarker' };
      var label_marker_assign = function () {
          return (
            (label_marker_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            label_marker_assign.apply(this, arguments)
          );
        },
        label_marker_rest = function (s, e) {
          var t = {};
          for (var p in s)
            Object.prototype.hasOwnProperty.call(s, p) && e.indexOf(p) < 0 && (t[p] = s[p]);
          if (null != s && 'function' == typeof Object.getOwnPropertySymbols) {
            var i = 0;
            for (p = Object.getOwnPropertySymbols(s); i < p.length; i++)
              e.indexOf(p[i]) < 0 &&
                Object.prototype.propertyIsEnumerable.call(s, p[i]) &&
                (t[p[i]] = s[p[i]]);
          }
          return t;
        };
      function LabelContainer(props) {
        var theme = (0, useTheme.A)(),
          sourceX = props.sourceX,
          sourceY = props.sourceY,
          sourceRadius = props.sourceRadius,
          contentWidth = props.contentWidth,
          contentHeight = props.contentHeight,
          _a = props.contentPadding,
          contentPadding = void 0 === _a ? 8 : _a,
          _b = props.contentBorderRadius,
          contentBorderRadius = void 0 === _b ? 2 : _b,
          _c = props.arrowLength,
          preferredArrowLength = void 0 === _c ? 8 : _c,
          _d = props.angle,
          angle = void 0 === _d ? -30 : _d,
          _e = props.repositionThreshold,
          repositionThreshold = void 0 === _e ? 200 : _e,
          children = props.children,
          otherProps = label_marker_rest(props, [
            'sourceX',
            'sourceY',
            'sourceRadius',
            'contentWidth',
            'contentHeight',
            'contentPadding',
            'contentBorderRadius',
            'arrowLength',
            'angle',
            'repositionThreshold',
            'children',
          ]),
          entityManager = react.useContext(EntityManagerContext),
          contentWidthOuter = contentWidth + 2 * contentPadding,
          contentHeightOuter = contentHeight + 2 * contentPadding,
          preferredLocation = react.useMemo(
            function () {
              var theta =
                  ((function () {
                    for (var cur = angle; cur > 180 || cur < -180; )
                      cur > 180 ? (cur -= 180) : (cur += 180);
                    return cur;
                  })() *
                    Math.PI) /
                  180,
                anchorX1 = sourceX + Math.cos(theta) * sourceRadius,
                anchorY1 = sourceY + Math.sin(theta) * sourceRadius,
                anchorX2 = anchorX1 + Math.cos(theta) * preferredArrowLength,
                anchorY2 = anchorY1 + Math.sin(theta) * preferredArrowLength,
                rectStart =
                  theta >= 0 && theta < Math.PI / 2
                    ? [anchorX2 - contentBorderRadius, anchorY2]
                    : theta >= Math.PI / 2 && theta < Math.PI
                      ? [anchorX2 - contentWidthOuter + contentBorderRadius, anchorY2]
                      : theta >= -Math.PI && theta < -Math.PI / 2
                        ? [
                            anchorX2 - contentWidthOuter + contentBorderRadius,
                            anchorY2 - contentHeightOuter,
                          ]
                        : [anchorX2 - contentBorderRadius, anchorY2 - contentHeightOuter];
              return {
                anchorX1,
                anchorX2,
                anchorY1,
                anchorY2,
                borderBBox: {
                  minX: rectStart[0],
                  minY: rectStart[1],
                  maxX: rectStart[0] + contentWidthOuter,
                  maxY: rectStart[1] + contentHeightOuter,
                },
              };
            },
            [
              angle,
              contentBorderRadius,
              contentWidthOuter,
              contentHeightOuter,
              preferredArrowLength,
              sourceRadius,
              sourceX,
              sourceY,
            ],
          ),
          _f = react.useState(null),
          labelLocation = _f[0],
          setLabelLocation = _f[1];
        return (
          react.useLayoutEffect(
            function () {
              var nonColliding = entityManager.getNonColliding(preferredLocation.borderBBox, {
                distLimit: repositionThreshold,
              });
              if (nonColliding) {
                var entity = entityManager.add({ bbox: nonColliding });
                if (nonColliding !== preferredLocation.borderBBox) {
                  var width = nonColliding.maxX - nonColliding.minX,
                    height = nonColliding.maxY - nonColliding.minY,
                    centerX = nonColliding.minX + width / 2,
                    centerY = nonColliding.minY + height / 2,
                    theta_1 = Math.atan2(centerY - sourceY, centerX - sourceX),
                    anchorX1 = sourceX + Math.cos(theta_1) * sourceRadius,
                    anchorY1 = sourceY + Math.sin(theta_1) * sourceRadius,
                    anchorX2 =
                      theta_1 >= -Math.PI / 2 && theta_1 < Math.PI / 2
                        ? nonColliding.minX + contentBorderRadius
                        : nonColliding.maxX - contentBorderRadius,
                    anchorY2 =
                      theta_1 >= 0 && theta_1 < Math.PI ? nonColliding.minY : nonColliding.maxY;
                  setLabelLocation({
                    anchorX1,
                    anchorY1,
                    anchorX2,
                    anchorY2,
                    borderBBox: nonColliding,
                  });
                } else setLabelLocation(preferredLocation);
                return function () {
                  entityManager.remove(entity);
                };
              }
            },
            [
              entityManager,
              preferredLocation,
              contentBorderRadius,
              sourceRadius,
              sourceX,
              sourceY,
              repositionThreshold,
            ],
          ),
          labelLocation &&
            react.createElement(
              'g',
              label_marker_assign({}, otherProps),
              react.createElement('line', {
                x1: labelLocation.anchorX1,
                y1: labelLocation.anchorY1,
                x2: labelLocation.anchorX2,
                y2: labelLocation.anchorY2,
              }),
              react.createElement('rect', {
                x: labelLocation.borderBBox.minX,
                y: labelLocation.borderBBox.minY,
                width: labelLocation.borderBBox.maxX - labelLocation.borderBBox.minX,
                height: labelLocation.borderBBox.maxY - labelLocation.borderBBox.minY,
                rx: contentBorderRadius,
                ry: contentBorderRadius,
                fill: theme.palette.background.paper,
              }),
              react.createElement(
                'g',
                {
                  transform: 'translate('
                    .concat(labelLocation.borderBBox.minX + contentPadding, ' ')
                    .concat(labelLocation.borderBBox.minY + contentPadding, ')'),
                },
                children,
              ),
            )
        );
      }
      var label_marker_classes_container = 'name-label-container',
        StyledLabelContainer = (0, styled.Ay)(function (props) {
          return react.createElement(LabelContainer, label_marker_assign({}, props));
        })(function (_a) {
          var _b,
            theme = _a.theme;
          return (
            ((_b = {})['&.'.concat(label_marker_classes_container)] = {
              fontSize: theme.typography.fontSize,
              fontFamily: theme.typography.fontFamily,
              userSelect: 'none',
            }),
            _b
          );
        }),
        Text = react.forwardRef(function (_a, ref) {
          var children = _a.children,
            otherProps = label_marker_rest(_a, ['children']);
          return react.createElement(
            'text',
            label_marker_assign(
              { ref, x: 0, y: 0, strokeWidth: 0, dominantBaseline: 'middle', textAnchor: 'middle' },
              otherProps,
            ),
            children,
          );
        });
      function NameLabel(props) {
        var theme = (0, useTheme.A)(),
          text = props.text,
          _a = props.contentPadding,
          contentPadding = void 0 === _a ? 4 : _a,
          className = props.className,
          otherProps = label_marker_rest(props, ['text', 'contentPadding', 'className']),
          _b = react.useState(0),
          contentWidth = _b[0],
          setContentWidth = _b[1],
          _c = react.useState(0),
          contentHeight = _c[0],
          setContentHeight = _c[1],
          _d = react.useState(!1),
          show = _d[0],
          setShow = _d[1],
          textRef = react.useRef(null);
        return (
          react.useEffect(
            function () {
              var updateContentSize = function () {
                if (textRef.current) {
                  var bbox = textRef.current.getBBox();
                  0 === bbox.width
                    ? requestAnimationFrame(function () {
                        updateContentSize();
                      })
                    : (setContentWidth(bbox.width + 16),
                      setContentHeight(bbox.height),
                      setShow(!0));
                }
              };
              updateContentSize();
            },
            [theme],
          ),
          react.createElement(
            react.Fragment,
            null,
            !show &&
              react.createElement(Text, { ref: textRef, style: { visibility: 'hidden' } }, text),
            show &&
              react.createElement(
                StyledLabelContainer,
                label_marker_assign(
                  {
                    contentWidth,
                    contentHeight,
                    contentPadding,
                    className: (0, clsx_m.default)(label_marker_classes_container, className),
                    stroke: theme.palette.info.main,
                    strokeWidth: 1,
                  },
                  otherProps,
                ),
                react.createElement(
                  Text,
                  {
                    fill: theme.palette.info.main,
                    transform: 'translate('
                      .concat(contentWidth / 2, ',')
                      .concat(contentHeight / 2, ')'),
                  },
                  text,
                ),
              ),
          )
        );
      }
      (function withAutoScaling(LabelComponent) {
        return function (_a) {
          var sourceX = _a.sourceX,
            sourceY = _a.sourceY,
            sourceRadius = _a.sourceRadius,
            arrowLength = _a.arrowLength,
            _b = _a.repositionThreshold,
            repositionThreshold = void 0 === _b ? 5 : _b,
            transform = _a.transform,
            otherProps = label_marker_rest(_a, [
              'sourceX',
              'sourceY',
              'sourceRadius',
              'arrowLength',
              'repositionThreshold',
              'transform',
            ]),
            scale = (function useAutoScale(factor, threshold) {
              void 0 === threshold && (threshold = DefaultMarkerActualSizeMinZoom);
              var leaflet = (0, context.jF)(),
                getScale = react.useCallback(
                  function () {
                    return !leaflet.map || leaflet.map.getZoom() >= threshold
                      ? 1
                      : factor / Math.pow(2, leaflet.map.getZoom());
                  },
                  [leaflet.map, threshold, factor],
                ),
                _a = react.useState(getScale),
                scale = _a[0],
                setScale = _a[1];
              return (
                react.useEffect(
                  function () {
                    leaflet.map &&
                      leaflet.map.on('zoom', function () {
                        return setScale(getScale());
                      });
                  },
                  [leaflet.map, getScale],
                ),
                scale
              );
            })(1.2, 1 / 0);
          return react.createElement(
            LabelComponent,
            label_marker_assign({}, otherProps, {
              sourceX: sourceX / scale,
              sourceY: sourceY / scale,
              sourceRadius: sourceRadius / scale,
              arrowLength: arrowLength ? arrowLength / scale : void 0,
              repositionThreshold: repositionThreshold / scale,
              transform: (0, clsx_m.default)(transform, 'scale('.concat(scale, ')')),
            }),
          );
        };
      })(NameLabel);
      (LabelContainer.__docgenInfo = {
        description: '',
        methods: [],
        displayName: 'LabelContainer',
      }),
        (NameLabel.__docgenInfo = { description: '', methods: [], displayName: 'NameLabel' });
      var workcell_marker_assign = function () {
          return (
            (workcell_marker_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            workcell_marker_assign.apply(this, arguments)
          );
        },
        workcell_marker_rest = function (s, e) {
          var t = {};
          for (var p in s)
            Object.prototype.hasOwnProperty.call(s, p) && e.indexOf(p) < 0 && (t[p] = s[p]);
          if (null != s && 'function' == typeof Object.getOwnPropertySymbols) {
            var i = 0;
            for (p = Object.getOwnPropertySymbols(s); i < p.length; i++)
              e.indexOf(p[i]) < 0 &&
                Object.prototype.propertyIsEnumerable.call(s, p[i]) &&
                (t[p[i]] = s[p[i]]);
          }
          return t;
        },
        workcell_marker_debug = src_browser_default()('Map:WorkcellMarker'),
        DefaultIcon = function (_a) {
          var cx = _a.cx,
            cy = _a.cy,
            size = _a.size,
            theme = (0, useTheme.A)();
          return react.createElement(
            'svg',
            {
              xmlns: 'http://www.w3.org/2000/svg',
              x: cx - 0.7 * size,
              y: cy - 0.7 * size,
              width: 1.4 * size,
              height: 1.4 * size,
              viewBox: '0 0 24 24',
            },
            react.createElement('path', {
              fill: theme.palette.success.main,
              d: 'M19 3H4.99c-1.11 0-1.98.9-1.98 2L3 19c0 1.1.88 2 1.99 2H19c1.1 0 2-.9 2-2V5c0-1.1-.9-2-2-2zm0 12h-4c0 1.66-1.35 3-3 3s-3-1.34-3-3H4.99V5H19v10zm-3-5h-2V7h-4v3H8l4 4 4-4z',
            }),
          );
        },
        workcell_marker_classes_text = 'workcell-marker-text',
        workcell_marker_classes_clickable = 'workcell-marker-clickable',
        workcell_marker_StyledG = (0, styled.Ay)('g')(function () {
          var _a;
          return (
            ((_a = {})['& .'.concat(workcell_marker_classes_text)] = {
              dominantBaseline: 'central',
              textAnchor: 'middle',
              fontSize: '0.18px',
              fontWeight: 'bold',
              fill: 'white',
              textShadow: '-1px 0 black, 0 1px black, 1px 0 black, 0 -1px black',
              pointerEvents: 'none',
              userSelect: 'none',
            }),
            (_a['& .'.concat(workcell_marker_classes_clickable)] = {
              pointerEvents: 'auto',
              cursor: 'pointer',
            }),
            _a
          );
        }),
        WorkcellMarker = react.forwardRef(function (props, ref) {
          workcell_marker_debug('render');
          var cx = props.cx,
            cy = props.cy,
            size = props.size,
            iconPath = props.iconPath,
            otherProps = workcell_marker_rest(props, ['cx', 'cy', 'size', 'iconPath']),
            _a = react.useState(!1),
            imageHasError = _a[0],
            setImageHasError = _a[1],
            useImageIcon = !!iconPath && !imageHasError;
          return react.createElement(
            workcell_marker_StyledG,
            workcell_marker_assign({ ref }, otherProps),
            react.createElement(
              'g',
              { className: otherProps.onClick && workcell_marker_classes_clickable },
              useImageIcon
                ? react.createElement('image', {
                    href: iconPath,
                    x: cx - size / 2,
                    y: cy - size / 2,
                    width: size,
                    height: size,
                    onError: function () {
                      return setImageHasError(!0);
                    },
                  })
                : react.createElement(DefaultIcon, workcell_marker_assign({}, props)),
              react.createElement('rect', {
                x: cx - size / 2,
                y: cy - size / 2,
                width: size,
                height: size,
                fill: 'transparent',
              }),
            ),
          );
        });
      WorkcellMarker.__docgenInfo = { description: '', methods: [], displayName: 'WorkcellMarker' };
      var cube_maker_assign = function () {
          return (
            (cube_maker_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            cube_maker_assign.apply(this, arguments)
          );
        },
        CubeMaker = function (_a, props) {
          var position = _a.position,
            size = _a.size,
            rot = _a.rot,
            color = _a.color,
            meshRef = _a.meshRef;
          return react.createElement(
            'mesh',
            cube_maker_assign({}, props, {
              position: [position[0], position[1], position[2]],
              rotation: rot,
              scale: [size[0], size[1], size[2]],
              ref: meshRef || null,
            }),
            react.createElement('planeGeometry', null),
            react.createElement('meshStandardMaterial', {
              color: color || 'black',
              opacity: 0.6,
              transparent: !0,
            }),
          );
        };
      CubeMaker.__docgenInfo = { description: '', methods: [], displayName: 'CubeMaker' };
      (function (_a) {
        var meshRef = _a.meshRef,
          door = _a.door,
          height = _a.height,
          color = _a.color,
          v1_x = door.v1_x,
          v1_y = door.v1_y,
          v2_x = door.v2_x,
          v2_y = door.v2_y,
          angle = Math.atan2(v1_y - v2_y, v1_x - v2_x) - Math.PI / 2,
          rot = new three_module.O9p(0, 0, angle),
          pos = (function (v1_x, v1_y, v2_x, v2_y) {
            return [(v2_x + v1_x) / 2, (v2_y + v1_y) / 2];
          })(v1_x, v1_y, v2_x, v2_y).concat(height / 2 + 0),
          dist = (function (v1_x, v1_y, v2_x, v2_y) {
            return Math.hypot(v2_x - v1_x, v2_y - v1_y);
          })(v1_x, v1_y, v2_x, v2_y);
        return react.createElement(CubeMaker, {
          meshRef,
          key: door.name,
          position: pos,
          size: [0.5, dist, height],
          rot,
          color,
        });
      }).__docgenInfo = { description: '', methods: [], displayName: 'DoorThreeMaker' };
      var Line = __webpack_require__(
          '../../node_modules/.pnpm/@react-three+drei@9.103.0_@react-three+fiber@8.16.1_@types+react@18.2.14_@types+three@0.156.0_qmmk6jl2a4mcpyts7yv5lqd3cy/node_modules/@react-three/drei/core/Line.js',
        ),
        shapes =
          (__webpack_require__(
            '../../node_modules/.pnpm/@react-three+drei@9.103.0_@react-three+fiber@8.16.1_@types+react@18.2.14_@types+three@0.156.0_qmmk6jl2a4mcpyts7yv5lqd3cy/node_modules/@react-three/drei/core/Text.js',
          ),
          __webpack_require__(
            '../../node_modules/.pnpm/@react-three+fiber@8.16.1_react-dom@18.2.0_react@18.2.0_three@0.156.1/node_modules/@react-three/fiber/dist/index-d98fd1c7.esm.js',
          ),
          __webpack_require__(
            '../../node_modules/.pnpm/@react-three+drei@9.103.0_@react-three+fiber@8.16.1_@types+react@18.2.14_@types+three@0.156.0_qmmk6jl2a4mcpyts7yv5lqd3cy/node_modules/@react-three/drei/core/shapes.js',
          )),
        CircleShape = function (_a) {
          var position = _a.position,
            rotation = _a.rotation,
            onRobotClick = _a.onRobotClick,
            robot = _a.robot,
            segment = _a.segment,
            rotatedX = position.x + 0.7 * Math.cos(rotation.z - Math.PI / 2),
            rotatedY = position.y + 0.7 * Math.sin(rotation.z - Math.PI / 2);
          return react.createElement(
            react.Fragment,
            null,
            react.createElement(
              shapes.jl,
              { args: [0.7, segment], position, rotation, onClick: onRobotClick },
              react.createElement('meshBasicMaterial', { color: robot.color }),
            ),
            react.createElement(Line.N, {
              points: [position.x, position.y, position.z, rotatedX, rotatedY, position.z],
              color: 'black',
              linewidth: 2,
            }),
          );
        };
      CircleShape.__docgenInfo = { description: '', methods: [], displayName: 'CircleShape' };
      var Html = __webpack_require__(
          '../../node_modules/.pnpm/@react-three+drei@9.103.0_@react-three+fiber@8.16.1_@types+react@18.2.14_@types+three@0.156.0_qmmk6jl2a4mcpyts7yv5lqd3cy/node_modules/@react-three/drei/web/Html.js',
        ),
        TextThreeRendering = function (_a) {
          var position = _a.position,
            text = _a.text,
            _b = react.useState(!1),
            isHovered = _b[0],
            setIsHovered = _b[1],
            scaleFactor = isHovered ? 2 : 1,
            positionX = text && text.length > 5 ? -2 : -1;
          return react.createElement(
            react.Fragment,
            null,
            react.createElement(
              'mesh',
              { position },
              react.createElement(
                'mesh',
                { position: [positionX, 0, 4] },
                text &&
                  react.createElement(
                    Html.E,
                    { zIndexRange: [0, 0, 1] },
                    text &&
                      react.createElement(
                        'div',
                        {
                          style: {
                            backgroundColor: 'rgba(255, 255, 255, 0.8)',
                            padding: '0.2rem 0.5rem',
                            borderRadius: '4px',
                            fontSize: '0.6rem',
                            transform: 'scale('.concat(scaleFactor, ')'),
                            transition: 'transform 0.6s cubic-bezier(0.2, 0.8, 0.2, 1)',
                          },
                          onPointerOver: function () {
                            setIsHovered(!0);
                          },
                          onPointerOut: function () {
                            setIsHovered(!1);
                          },
                        },
                        text,
                      ),
                  ),
              ),
            ),
          );
        };
      TextThreeRendering.__docgenInfo = {
        description: '',
        methods: [],
        displayName: 'TextThreeRendering',
      };
      var Tabs = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Tabs/Tabs.js',
        ),
        navigation_bar_assign = function () {
          return (
            (navigation_bar_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            navigation_bar_assign.apply(this, arguments)
          );
        },
        navigation_bar_classes_tabsContainer = 'navigation-bar-root',
        simple_filter_classes_simpleFilter =
          ((0, styled.Ay)(function (props) {
            return react.createElement(Tabs.A, navigation_bar_assign({}, props));
          })(function () {
            var _a;
            return (
              ((_a = {})['&.'.concat(navigation_bar_classes_tabsContainer)] = { flexGrow: 4 }), _a
            );
          }),
          'simple-filter-root'),
        simple_filter_classes_filterBar = 'simple-filter-filterbar',
        simple_filter_classes_divider = 'simple-filter-divider',
        List =
          ((0, styled.Ay)('div')(function (_a) {
            var _b,
              theme = _a.theme;
            return (
              ((_b = {})['&.'.concat(simple_filter_classes_simpleFilter)] = {
                margin: '1rem',
                borderColor: theme.palette.success.main,
              }),
              (_b['& .'.concat(simple_filter_classes_filterBar)] = { width: '100%' }),
              (_b['& .'.concat(simple_filter_classes_divider)] = { margin: '1.5rem 0' }),
              _b
            );
          }),
          __webpack_require__(
            '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/List/List.js',
          )),
        ListItem = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/ListItem/ListItem.js',
        ),
        simple_info_assign = function () {
          return (
            (simple_info_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            simple_info_assign.apply(this, arguments)
          );
        },
        simple_info_rest = function (s, e) {
          var t = {};
          for (var p in s)
            Object.prototype.hasOwnProperty.call(s, p) && e.indexOf(p) < 0 && (t[p] = s[p]);
          if (null != s && 'function' == typeof Object.getOwnPropertySymbols) {
            var i = 0;
            for (p = Object.getOwnPropertySymbols(s); i < p.length; i++)
              e.indexOf(p[i]) < 0 &&
                Object.prototype.propertyIsEnumerable.call(s, p[i]) &&
                (t[p[i]] = s[p[i]]);
          }
          return t;
        },
        simple_info_classes_container = 'simpleinfo-container',
        simple_info_classes_tableRow = 'simpleinfo-table-row',
        simple_info_classes_displayName = 'simpleinfo-display-name',
        simple_info_classes_value = 'simpleinfo-value',
        simple_info_classes_arrayListItem = 'simpleinfo-array-list-item',
        simple_info_classes_arrayItemValue = 'simpleinfo-array-item-value',
        simple_info_classes_disabled = 'simpleinfo-disabled',
        simple_info_StyledDiv = (0, styled.Ay)('div')(function (_a) {
          var _b,
            theme = _a.theme;
          return (
            ((_b = {})['& .'.concat(simple_info_classes_container)] = {
              display: 'table',
              borderCollapse: 'collapse',
              width: '100%',
              overflowX: 'auto',
            }),
            (_b['& .'.concat(simple_info_classes_tableRow)] = { display: 'table-row' }),
            (_b['& .'.concat(simple_info_classes_displayName)] = {
              display: 'table-cell',
              borderBottom: '1px solid',
              borderBottomColor: theme.palette.divider,
              borderTop: '1px solid',
              borderTopColor: theme.palette.divider,
              background: theme.palette.action.hover,
              padding: theme.spacing(0.25, 2),
              width: '30%',
            }),
            (_b['& .'.concat(simple_info_classes_value)] = {
              display: 'table-cell',
              textAlign: 'end',
              borderBottom: '1px solid',
              borderBottomColor: theme.palette.divider,
              borderTop: '1px solid',
              borderTopColor: theme.palette.divider,
              padding: theme.spacing(0.25, 2),
            }),
            (_b['& .'.concat(simple_info_classes_arrayListItem)] = { justifyContent: 'flex-end' }),
            (_b['& .'.concat(simple_info_classes_arrayItemValue)] = { textAlign: 'end' }),
            (_b['& .'.concat(simple_info_classes_disabled)] = {
              color: theme.palette.action.disabled,
            }),
            _b
          );
        });
      (function (props) {
        var infoData = props.infoData,
          overrideStyle = props.overrideStyle,
          otherProps = simple_info_rest(props, ['infoData', 'overrideStyle']),
          renderLine = function (data) {
            switch (typeof data.value) {
              case 'object':
                if (Array.isArray(data.value))
                  return (function (_a) {
                    var name = _a.name,
                      value = _a.value,
                      className = _a.className,
                      disabled = _a.disabled,
                      arrayItemValueStyle = (
                        null == className ? void 0 : className.overrideArrayItemValue
                      )
                        ? null == className
                          ? void 0
                          : className.overrideArrayItemValue
                        : simple_info_classes_arrayItemValue,
                      valueStyle = (null == className ? void 0 : className.overrideValue)
                        ? null == className
                          ? void 0
                          : className.overrideValue
                        : simple_info_classes_value;
                    return react.createElement(
                      react.Fragment,
                      null,
                      react.createElement(
                        Typography.A,
                        {
                          className: simple_info_classes_displayName,
                          variant: 'body1',
                          role: 'rowheader',
                        },
                        name,
                      ),
                      react.createElement(
                        List.A,
                        { className: valueStyle, dense: !0, role: 'cell' },
                        value.map(function (item, i) {
                          return react.createElement(
                            ListItem.Ay,
                            { key: i, className: simple_info_classes_arrayListItem },
                            react.createElement(
                              Typography.A,
                              {
                                variant: 'body1',
                                className: (0, clsx_m.default)(
                                  arrayItemValueStyle,
                                  disabled ? simple_info_classes_disabled : void 0,
                                  Array.isArray(null == className ? void 0 : className.value)
                                    ? null == className
                                      ? void 0
                                      : className.value[i]
                                    : null == className
                                      ? void 0
                                      : className.value,
                                ),
                              },
                              item,
                            ),
                          );
                        }),
                      ),
                    );
                  })(data);
                throw Error('nested object is not supported');
              case 'function':
              case 'symbol':
                break;
              default:
                return (function (_a) {
                  var name = _a.name,
                    value = _a.value,
                    className = _a.className,
                    disabled = _a.disabled,
                    wrap = _a.wrap;
                  return react.createElement(
                    react.Fragment,
                    null,
                    react.createElement(
                      Typography.A,
                      {
                        className: simple_info_classes_displayName,
                        variant: 'body1',
                        role: 'rowheader',
                      },
                      name,
                    ),
                    react.createElement(
                      Typography.A,
                      {
                        noWrap: !wrap,
                        variant: 'body1',
                        className: (0, clsx_m.default)(
                          (null == className ? void 0 : className.overrideValue)
                            ? null == className
                              ? void 0
                              : className.overrideValue
                            : simple_info_classes_value,
                          disabled ? simple_info_classes_disabled : void 0,
                          null == className ? void 0 : className.value,
                        ),
                        role: 'cell',
                      },
                      value,
                    ),
                  );
                })(data);
            }
          };
        return react.createElement(
          simple_info_StyledDiv,
          simple_info_assign({}, otherProps),
          react.createElement(
            'div',
            {
              className: (null == overrideStyle ? void 0 : overrideStyle.container)
                ? null == overrideStyle
                  ? void 0
                  : overrideStyle.container
                : simple_info_classes_container,
              role: 'table',
            },
            infoData.map(function (item) {
              return react.createElement(
                react.Fragment,
                { key: item.name },
                react.createElement(
                  'div',
                  {
                    className: (null == overrideStyle ? void 0 : overrideStyle.tableRow)
                      ? null == overrideStyle
                        ? void 0
                        : overrideStyle.tableRow
                      : simple_info_classes_tableRow,
                    role: 'row',
                    'aria-label': item.name,
                  },
                  renderLine(item),
                ),
              );
            }),
          ),
        );
      }).__docgenInfo = { description: '', methods: [], displayName: 'SimpleInfo' };
      var status_label_classes_status = 'status-label-root',
        status_label_classes_unknown = 'status-label-unknown',
        svg_text_assign =
          ((0, styled.Ay)('div')(function (_a) {
            var _b,
              theme = _a.theme;
            return (
              ((_b = {})['&.'.concat(status_label_classes_status)] = {
                borderColor: theme.palette.primary.main,
                borderRadius: theme.shape.borderRadius,
                borderStyle: 'solid',
                borderWidth: '2px',
                padding: '5px',
                width: '4rem',
                textAlign: 'center',
                flexShrink: 0,
              }),
              (_b['&.'.concat(status_label_classes_unknown)] = {
                borderColor: theme.palette.grey[500],
              }),
              _b
            );
          }),
          function () {
            return (
              (svg_text_assign =
                Object.assign ||
                function (t) {
                  for (var s, i = 1, n = arguments.length; i < n; i++)
                    for (var p in (s = arguments[i]))
                      Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                  return t;
                }),
              svg_text_assign.apply(this, arguments)
            );
          }),
        svg_text_rest = function (s, e) {
          var t = {};
          for (var p in s)
            Object.prototype.hasOwnProperty.call(s, p) && e.indexOf(p) < 0 && (t[p] = s[p]);
          if (null != s && 'function' == typeof Object.getOwnPropertySymbols) {
            var i = 0;
            for (p = Object.getOwnPropertySymbols(s); i < p.length; i++)
              e.indexOf(p[i]) < 0 &&
                Object.prototype.propertyIsEnumerable.call(s, p[i]) &&
                (t[p[i]] = s[p[i]]);
          }
          return t;
        },
        SvgText = function (props) {
          var text = props.text,
            targetWidth = props.targetWidth,
            otherProps = svg_text_rest(props, ['text', 'targetWidth']);
          return react.createElement(
            'text',
            svg_text_assign(
              {
                ref: function (textElem) {
                  if (textElem)
                    for (
                      textElem.textContent = text;
                      textElem.getComputedTextLength() > targetWidth;

                    )
                      textElem.textContent =
                        textElem.textContent.slice(0, textElem.textContent.length - 6) + '…';
                },
              },
              otherProps,
            ),
          );
        };
      SvgText.__docgenInfo = {
        description:
          'A wrapper to `<text>` element that attempts to fix text into a given width, ellipsing it if it\nis too long. Unlike the `textLength` attribute, this does not "compress" or "expand" the text.\n@param props',
        methods: [],
        displayName: 'SvgText',
      };
      __webpack_require__(
        '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/Create.js',
      ),
        __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/Delete.js',
        ),
        __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/PlaceOutlined.js',
        );
      var ScheduleUntilValue,
        IconButton = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/IconButton/IconButton.js',
        ),
        ListItemIcon = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/ListItemIcon/ListItemIcon.js',
        ),
        ListItemText = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/ListItemText/ListItemText.js',
        ),
        create_task_assign =
          (__webpack_require__(
            '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/ListItemSecondaryAction/ListItemSecondaryAction.js',
          ),
          __webpack_require__(
            '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Chip/Chip.js',
          ),
          __webpack_require__(
            '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/MenuItem/MenuItem.js',
          ),
          __webpack_require__(
            '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/FormControl/FormControl.js',
          ),
          __webpack_require__(
            '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/FormHelperText/FormHelperText.js',
          ),
          __webpack_require__(
            '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/RadioGroup/RadioGroup.js',
          ),
          __webpack_require__(
            '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/FormControlLabel/FormControlLabel.js',
          ),
          __webpack_require__(
            '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Radio/Radio.js',
          ),
          __webpack_require__(
            '../../node_modules/.pnpm/@mui+x-date-pickers@5.0.20_@emotion+react@11.9.3_@emotion+styled@11.9.3_@mui+material@5.8.7_@_kkp62yhptrthrgtgtakbrwpa6y/node_modules/@mui/x-date-pickers/DateTimePicker/DateTimePicker.js',
          ),
          __webpack_require__(
            '../../node_modules/.pnpm/@mui+x-date-pickers@5.0.20_@emotion+react@11.9.3_@emotion+styled@11.9.3_@mui+material@5.8.7_@_kkp62yhptrthrgtgtakbrwpa6y/node_modules/@mui/x-date-pickers/DatePicker/DatePicker.js',
          ),
          __webpack_require__(
            '../../node_modules/.pnpm/@mui+x-date-pickers@5.0.20_@emotion+react@11.9.3_@emotion+styled@11.9.3_@mui+material@5.8.7_@_kkp62yhptrthrgtgtakbrwpa6y/node_modules/@mui/x-date-pickers/TimePicker/TimePicker.js',
          ),
          function () {
            return (
              (create_task_assign =
                Object.assign ||
                function (t) {
                  for (var s, i = 1, n = arguments.length; i < n; i++)
                    for (var p in (s = arguments[i]))
                      Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                  return t;
                }),
              create_task_assign.apply(this, arguments)
            );
          }),
        create_task_classes = {
          title: 'dialogue-info-value',
          selectFileBtn: 'create-task-selected-file-btn',
          taskList: 'create-task-task-list',
          selectedTask: 'create-task-selected-task',
          actionBtn: 'dialogue-action-button',
        };
      (0, styled.Ay)(function (props) {
        return react.createElement(Dialog.A, create_task_assign({}, props));
      })(function (_a) {
        var _b,
          theme = _a.theme;
        return (
          ((_b = {})['& .'.concat(create_task_classes.selectFileBtn)] = {
            marginBottom: theme.spacing(1),
          }),
          (_b['& .'.concat(create_task_classes.taskList)] = {
            flex: '1 1 auto',
            minHeight: 400,
            maxHeight: '50vh',
            overflow: 'auto',
          }),
          (_b['& .'.concat(create_task_classes.selectedTask)] = {
            background: theme.palette.action.focus,
          }),
          (_b['& .'.concat(create_task_classes.title)] = { flex: '1 1 auto' }),
          (_b['& .'.concat(create_task_classes.actionBtn)] = { minWidth: 80 }),
          _b
        );
      });
      !(function (ScheduleUntilValue) {
        (ScheduleUntilValue.NEVER = 'never'), (ScheduleUntilValue.ON = 'on');
      })(ScheduleUntilValue || (ScheduleUntilValue = {}));
      var ChevronRight = __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/ChevronRight.js',
        ),
        ExpandMore = __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/ExpandMore.js',
        ),
        Timeline = __webpack_require__(
          '../../node_modules/.pnpm/@mui+lab@5.0.0-alpha.86_@emotion+react@11.9.3_@emotion+styled@11.9.3_@mui+material@5.8.7_@typ_73irs6jmm2f7meo6ik74k3gbze/node_modules/@mui/lab/Timeline/Timeline.js',
        ),
        TreeItem = __webpack_require__(
          '../../node_modules/.pnpm/@mui+lab@5.0.0-alpha.86_@emotion+react@11.9.3_@emotion+styled@11.9.3_@mui+material@5.8.7_@typ_73irs6jmm2f7meo6ik74k3gbze/node_modules/@mui/lab/TreeItem/TreeItem.js',
        ),
        TimelineItem = __webpack_require__(
          '../../node_modules/.pnpm/@mui+lab@5.0.0-alpha.86_@emotion+react@11.9.3_@emotion+styled@11.9.3_@mui+material@5.8.7_@typ_73irs6jmm2f7meo6ik74k3gbze/node_modules/@mui/lab/TimelineItem/TimelineItem.js',
        ),
        TimelineOppositeContent = __webpack_require__(
          '../../node_modules/.pnpm/@mui+lab@5.0.0-alpha.86_@emotion+react@11.9.3_@emotion+styled@11.9.3_@mui+material@5.8.7_@typ_73irs6jmm2f7meo6ik74k3gbze/node_modules/@mui/lab/TimelineOppositeContent/TimelineOppositeContent.js',
        ),
        TimelineSeparator = __webpack_require__(
          '../../node_modules/.pnpm/@mui+lab@5.0.0-alpha.86_@emotion+react@11.9.3_@emotion+styled@11.9.3_@mui+material@5.8.7_@typ_73irs6jmm2f7meo6ik74k3gbze/node_modules/@mui/lab/TimelineSeparator/TimelineSeparator.js',
        ),
        TimelineDot = __webpack_require__(
          '../../node_modules/.pnpm/@mui+lab@5.0.0-alpha.86_@emotion+react@11.9.3_@emotion+styled@11.9.3_@mui+material@5.8.7_@typ_73irs6jmm2f7meo6ik74k3gbze/node_modules/@mui/lab/TimelineDot/TimelineDot.js',
        ),
        TimelineConnector = __webpack_require__(
          '../../node_modules/.pnpm/@mui+lab@5.0.0-alpha.86_@emotion+react@11.9.3_@emotion+styled@11.9.3_@mui+material@5.8.7_@typ_73irs6jmm2f7meo6ik74k3gbze/node_modules/@mui/lab/TimelineConnector/TimelineConnector.js',
        ),
        TimelineContent = __webpack_require__(
          '../../node_modules/.pnpm/@mui+lab@5.0.0-alpha.86_@emotion+react@11.9.3_@emotion+styled@11.9.3_@mui+material@5.8.7_@typ_73irs6jmm2f7meo6ik74k3gbze/node_modules/@mui/lab/TimelineContent/TimelineContent.js',
        ),
        TreeView = __webpack_require__(
          '../../node_modules/.pnpm/@mui+lab@5.0.0-alpha.86_@emotion+react@11.9.3_@emotion+styled@11.9.3_@mui+material@5.8.7_@typ_73irs6jmm2f7meo6ik74k3gbze/node_modules/@mui/lab/TreeView/TreeView.js',
        ),
        task_timeline_assign = function () {
          return (
            (task_timeline_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            task_timeline_assign.apply(this, arguments)
          );
        },
        task_timeline_classes = {
          paper: 'timeline-paper',
          secondaryTail: 'timeline-secondary-tail',
          timelineRoot: 'timeline-root',
        },
        StyledTimeLine = (0, styled.Ay)(function (props) {
          return react.createElement(Timeline.A, task_timeline_assign({}, props));
        })(function (_a) {
          var _b,
            theme = _a.theme;
          return (
            ((_b = {})['& .'.concat(task_timeline_classes.paper)] = {
              padding: theme.spacing(1),
              marginTop: theme.spacing(1),
              width: '200px',
              overflow: 'auto',
              display: 'inline-block',
              maxHeight: '95%',
            }),
            (_b['& .'.concat(task_timeline_classes.secondaryTail)] = {
              backgroundColor: theme.palette.secondary.main,
            }),
            (_b['&.'.concat(task_timeline_classes.timelineRoot)] = { padding: '6px 0px' }),
            _b
          );
        });
      function NestedEvents(eventStates, eventId) {
        if (null != eventStates && void 0 !== eventId) {
          var event_1 = eventStates[eventId];
          if (void 0 !== event_1)
            return react.createElement(
              TreeItem.A,
              {
                nodeId: 'event-'.concat(event_1.id),
                key: 'event-'.concat(event_1.id),
                label: event_1.name || 'undefined',
              },
              event_1.deps
                ? event_1.deps.map(function (childId) {
                    return NestedEvents(eventStates, childId);
                  })
                : null,
            );
        }
        return null;
      }
      function colorDot(phase) {
        if (null == phase) return 'error';
        if (null == phase.final_event_id || null == phase.events) return 'grey';
        var root_event = phase.events[phase.final_event_id];
        if (null == root_event) return 'error';
        if (null == root_event.status) return 'error';
        switch (root_event.status) {
          case lib.ApiServerModelsRmfApiTaskStateStatus.Uninitialized:
          case lib.ApiServerModelsRmfApiTaskStateStatus.Blocked:
          case lib.ApiServerModelsRmfApiTaskStateStatus.Error:
          case lib.ApiServerModelsRmfApiTaskStateStatus.Failed:
            return 'error';
          case lib.ApiServerModelsRmfApiTaskStateStatus.Queued:
          case lib.ApiServerModelsRmfApiTaskStateStatus.Standby:
            return 'grey';
          case lib.ApiServerModelsRmfApiTaskStateStatus.Underway:
            return 'success';
          case lib.ApiServerModelsRmfApiTaskStateStatus.Skipped:
          case lib.ApiServerModelsRmfApiTaskStateStatus.Canceled:
          case lib.ApiServerModelsRmfApiTaskStateStatus.Killed:
            return 'secondary';
          case lib.ApiServerModelsRmfApiTaskStateStatus.Delayed:
            return 'warning';
          case lib.ApiServerModelsRmfApiTaskStateStatus.Completed:
            return 'primary';
          default:
            return 'error';
        }
      }
      function RenderPhase(phase) {
        return react.createElement(
          TimelineItem.A,
          { key: phase.id, sx: { width: 0 } },
          react.createElement(
            TimelineOppositeContent.A,
            { color: 'text.secondary' },
            react.createElement(
              Typography.A,
              { variant: 'overline', color: 'textSecondary', style: { textAlign: 'justify' } },
              null != phase.unix_millis_start_time
                ? new Date(phase.unix_millis_start_time).toLocaleTimeString()
                : null,
            ),
          ),
          react.createElement(
            TimelineSeparator.A,
            null,
            react.createElement(TimelineDot.A, { color: colorDot(phase) }),
            react.createElement(TimelineConnector.A, null),
          ),
          react.createElement(
            TimelineContent.A,
            null,
            react.createElement(
              Typography.A,
              { variant: 'overline', color: 'textSecondary', style: { textAlign: 'justify' } },
              phase.id,
              '. ',
              phase.category,
            ),
            react.createElement(
              TreeView.A,
              {
                defaultCollapseIcon: react.createElement(ExpandMore.A, null),
                defaultExpandIcon: react.createElement(ChevronRight.A, null),
              },
              phase.events && NestedEvents(phase.events, phase.final_event_id),
            ),
          ),
        );
      }
      function TaskTimeline(_a) {
        var taskState = _a.taskState,
          phases = taskState.phases ? Object.values(taskState.phases) : [];
        return react.createElement(
          StyledTimeLine,
          { className: task_timeline_classes.timelineRoot },
          phases.map(function (phase) {
            return RenderPhase(phase);
          }),
        );
      }
      TaskTimeline.__docgenInfo = { description: '', methods: [], displayName: 'TaskTimeline' };
      var task_info_classes = { infoValue: 'task-info-info-value' };
      (0, styled.Ay)('div')(function () {
        var _a;
        return (
          ((_a = {})['& .'.concat(task_info_classes.infoValue)] = {
            float: 'right',
            textAlign: 'right',
          }),
          _a
        );
      });
      var Paper = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Paper/Paper.js',
        ),
        task_logs_assign = function () {
          return (
            (task_logs_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            task_logs_assign.apply(this, arguments)
          );
        },
        task_logs_classes = { root: ''.concat('task-logs', '-root') },
        task_table_assign =
          ((0, styled.Ay)(function (props) {
            return react.createElement(Paper.A, task_logs_assign({ variant: 'outlined' }, props));
          })(function (_a) {
            var _b,
              theme = _a.theme;
            return (
              ((_b = {})['&.'.concat(task_logs_classes.root)] = {
                padding: theme.spacing(1),
                width: '100%',
                flex: '0 0 auto',
                maxHeight: '100%',
                overflow: 'auto',
              }),
              _b
            );
          }),
          __webpack_require__(
            '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/esm/ArrowCircleDown.js',
          ),
          __webpack_require__(
            '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/esm/ArrowCircleUp.js',
          ),
          function () {
            return (
              (task_table_assign =
                Object.assign ||
                function (t) {
                  for (var s, i = 1, n = arguments.length; i < n; i++)
                    for (var p in (s = arguments[i]))
                      Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                  return t;
                }),
              task_table_assign.apply(this, arguments)
            );
          }),
        task_table_classes = {
          taskRowHover: 'task-table-taskrow-hover',
          infoRow: 'task-table-info-row',
          phasesCell: 'task-table-phase-cell',
          phasesRow: 'task-table-phase-row',
          taskActiveCell: 'task-table-active-cell',
          taskCancelledCell: 'task-table-cancelled-cell',
          taskCompletedCell: 'task-table-completed-cell',
          taskFailedCell: 'task-table-failed-cell',
          taskPendingCell: 'task-table-pending-cell',
          taskQueuedCell: 'task-table-queued-cell',
          taskUnknownCell: 'task-table-unknown-cell',
        };
      (0, styled.Ay)(function (props) {
        return react.createElement(Table.A, task_table_assign({}, props));
      })(function (_a) {
        var _b,
          theme = _a.theme;
        return (
          ((_b = {})['& .'.concat(task_table_classes.taskRowHover)] = {
            background: theme.palette.action.hover,
            cursor: 'pointer',
          }),
          (_b['& .'.concat(task_table_classes.infoRow)] = { '& > *': { borderBottom: 'unset' } }),
          (_b['& .'.concat(task_table_classes.phasesCell)] = {
            padding: '0 '.concat(theme.spacing(1), 'px 0 ').concat(theme.spacing(1), 'px'),
            boxShadow: ''.concat(theme.shadows[1]),
            '&:last-child': { paddingRight: ''.concat(theme.spacing(1), 'px') },
          }),
          (_b['& .'.concat(task_table_classes.phasesRow)] = {
            marginBottom: theme.spacing(1),
            marginTop: theme.spacing(1),
          }),
          (_b['& .'.concat(task_table_classes.taskActiveCell)] = {
            backgroundColor: theme.palette.success.light,
            color: theme.palette.getContrastText(theme.palette.success.light),
          }),
          (_b['& .'.concat(task_table_classes.taskCancelledCell)] = {
            backgroundColor: theme.palette.grey[500],
            color: theme.palette.getContrastText(theme.palette.grey[500]),
          }),
          (_b['& .'.concat(task_table_classes.taskCompletedCell)] = {
            backgroundColor: theme.palette.info.light,
            color: theme.palette.getContrastText(theme.palette.info.light),
          }),
          (_b['& .'.concat(task_table_classes.taskFailedCell)] = {
            backgroundColor: theme.palette.error.main,
            color: theme.palette.getContrastText(theme.palette.error.main),
          }),
          (_b['& .'.concat(task_table_classes.taskQueuedCell)] = {
            backgroundColor: theme.palette.grey[300],
            color: theme.palette.getContrastText(theme.palette.grey[300]),
          }),
          (_b['& .'.concat(task_table_classes.taskUnknownCell)] = {
            backgroundColor: theme.palette.warning.main,
            color: theme.palette.getContrastText(theme.palette.warning.main),
          }),
          _b
        );
      });
      __webpack_require__(
        '../../node_modules/.pnpm/@mui+x-data-grid@5.17.26_@mui+material@5.8.7_@mui+system@5.8.7_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/x-data-grid/colDef/gridStringOperators.js',
      ),
        __webpack_require__(
          '../../node_modules/.pnpm/@mui+x-data-grid@5.17.26_@mui+material@5.8.7_@mui+system@5.8.7_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/x-data-grid/colDef/gridDateOperators.js',
        ),
        __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Stack/Stack.js',
        );
      var Tooltip = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Tooltip/Tooltip.js',
        ),
        task_table_datagrid_classes_taskActiveCell =
          (__webpack_require__(
            '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/esm/InsertInvitation.js',
          ),
          __webpack_require__(
            '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/esm/Person.js',
          ),
          'MuiDataGrid-cell-active-cell'),
        task_table_datagrid_classes_taskCancelledCell = 'MuiDataGrid-cell-cancelled-cell',
        task_table_datagrid_classes_taskCompletedCell = 'MuiDataGrid-cell-completed-cell',
        task_table_datagrid_classes_taskFailedCell = 'MuiDataGrid-cell-failed-cell',
        task_table_datagrid_classes_taskQueuedCell = 'MuiDataGrid-cell-queued-cell',
        task_table_datagrid_classes_taskUnknownCell = 'MuiDataGrid-cell-unknown-cell';
      (0, styled.Ay)(DataGrid.z)(function (_a) {
        var _b,
          theme = _a.theme;
        return (
          ((_b = {})['& .'.concat(task_table_datagrid_classes_taskActiveCell)] = {
            backgroundColor: theme.palette.success.light,
            color: theme.palette.getContrastText(theme.palette.success.light),
          }),
          (_b['& .'.concat(task_table_datagrid_classes_taskCancelledCell)] = {
            backgroundColor: theme.palette.grey[500],
            color: theme.palette.getContrastText(theme.palette.grey[500]),
          }),
          (_b['& .'.concat(task_table_datagrid_classes_taskCompletedCell)] = {
            backgroundColor: theme.palette.info.light,
            color: theme.palette.getContrastText(theme.palette.info.light),
          }),
          (_b['& .'.concat(task_table_datagrid_classes_taskFailedCell)] = {
            backgroundColor: theme.palette.error.main,
            color: theme.palette.getContrastText(theme.palette.error.main),
          }),
          (_b['& .'.concat(task_table_datagrid_classes_taskQueuedCell)] = {
            backgroundColor: theme.palette.grey[300],
            color: theme.palette.getContrastText(theme.palette.grey[300]),
          }),
          (_b['& .'.concat(task_table_datagrid_classes_taskUnknownCell)] = {
            backgroundColor: theme.palette.warning.main,
            color: theme.palette.getContrastText(theme.palette.warning.main),
          }),
          _b
        );
      });
      var createTheme = __webpack_require__(
        '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/styles/createTheme.js',
      );
      const common_theme = {
        secondary: { main: '#BC477B', light: '#F178AA', dark: '#880E4F' },
        success: { main: '#7FB800', light: '#B3EB49', dark: '#6dff6f' },
        error: { main: '#EF5264', light: '#FF8591', dark: '#B7153A' },
        warning: { main: '#FFB400', light: '#FFE64C', dark: '#C68500' },
        info: { main: '#64D4EE', light: '#64E9EE', dark: '#009FAF' },
      };
      var rmf_dark_assign = function () {
          return (
            (rmf_dark_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            rmf_dark_assign.apply(this, arguments)
          );
        },
        base = (0, createTheme.A)({
          palette: rmf_dark_assign(rmf_dark_assign({ mode: 'dark' }, common_theme), {
            primary: { main: '#37474F', dark: '#102027', light: '#62727B' },
            background: { default: '#102027', paper: '#62727B' },
          }),
        }),
        rmf_light_assign =
          ((0, createTheme.A)(
            {
              components: {
                MuiTableCell: {
                  styleOverrides: { stickyHeader: { backgroundColor: base.palette.primary.main } },
                },
              },
            },
            base,
          ),
          base.palette.text.primary,
          base.palette.background.paper,
          base.palette.text.primary,
          base.palette.background.paper,
          base.palette.text.primary,
          base.palette.text.primary,
          function () {
            return (
              (rmf_light_assign =
                Object.assign ||
                function (t) {
                  for (var s, i = 1, n = arguments.length; i < n; i++)
                    for (var p in (s = arguments[i]))
                      Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                  return t;
                }),
              rmf_light_assign.apply(this, arguments)
            );
          }),
        rmf_light_base = (0, createTheme.A)({
          palette: rmf_light_assign(rmf_light_assign({}, common_theme), {
            mode: 'light',
            primary: { main: '#1FD8DC ', light: '#ECEFF1', dark: '#B0BEC5' },
            background: { default: '#EEEEEE', paper: '#FFFFFF' },
          }),
        }),
        tooltip_classes_tooltipWidth =
          ((0, createTheme.A)(
            rmf_light_assign(rmf_light_assign({}, rmf_light_base), {
              components: {
                MuiTableCell: {
                  styleOverrides: {
                    stickyHeader: { backgroundColor: rmf_light_base.palette.primary.main },
                  },
                },
              },
            }),
          ),
          'tooltip-width'),
        tooltip_StyledDiv = (0, styled.Ay)('div')(function () {
          var _a;
          return ((_a = {})['& .'.concat(tooltip_classes_tooltipWidth)] = { maxWidth: 200 }), _a;
        }),
        tooltip_Tooltip = function (props) {
          var title = props.title,
            id = props.id,
            enabled = props.enabled;
          return react.createElement(
            tooltip_StyledDiv,
            null,
            enabled &&
              react.createElement(
                Tooltip.A,
                {
                  title,
                  arrow: !0,
                  id,
                  className: tooltip_classes_tooltipWidth,
                  'data-testid': id + '-tooltip',
                },
                props.children,
              ),
            !enabled && props.children,
          );
        };
      tooltip_Tooltip.__docgenInfo = { description: '', methods: [], displayName: 'Tooltip' };
      var CardHeader = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/CardHeader/CardHeader.js',
        ),
        Checkbox = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Checkbox/Checkbox.js',
        ),
        transfer_list_assign = function () {
          return (
            (transfer_list_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            transfer_list_assign.apply(this, arguments)
          );
        },
        transfer_list_classes = {
          container: 'transfer-list-container',
          cardHeader: 'transfer-list-card-header',
          list: 'transfer-list-list',
          button: 'transfer-list-button',
          transferControls: 'transfer-list-controls',
          cardContainer: 'transfer-list-card-container',
        },
        StyledGrid = (0, styled.Ay)(function (props) {
          return react.createElement(Grid.Ay, transfer_list_assign({}, props));
        })(function (_a) {
          var _b,
            theme = _a.theme;
          return (
            ((_b = {})['&.'.concat(transfer_list_classes.container)] = {
              height: '100%',
              '& > *': { height: '100%', flex: '1 1 0' },
            }),
            (_b['& .'.concat(transfer_list_classes.cardContainer)] = {
              height: '100%',
              '& > *': { height: '100%', flex: '1 1 0' },
            }),
            (_b['& .'.concat(transfer_list_classes.cardHeader)] = { padding: theme.spacing(1, 2) }),
            (_b['& .'.concat(transfer_list_classes.list)] = {
              backgroundColor: theme.palette.background.paper,
              overflow: 'auto',
            }),
            (_b['& .'.concat(transfer_list_classes.button)] = { margin: theme.spacing(0.5, 0) }),
            (_b['& .'.concat(transfer_list_classes.transferControls)] = {
              marginTop: 'auto',
              marginBottom: 'auto',
              height: 'auto',
              flex: '0 0 auto',
            }),
            _b
          );
        });
      function CustomList(_a) {
        var title = _a.title,
          items = _a.items,
          checked = _a.checked,
          setChecked = _a.setChecked,
          numberOfChecked = checked.size,
          handleToggleAllClick = react.useCallback(
            function () {
              numberOfChecked < items.length
                ? setChecked(new Set(items.values()))
                : setChecked(new Set());
            },
            [items, numberOfChecked, setChecked],
          );
        return react.createElement(
          Card.A,
          { variant: 'outlined', className: transfer_list_classes.cardContainer },
          react.createElement(
            Grid.Ay,
            { container: !0, direction: 'column', wrap: 'nowrap' },
            react.createElement(CardHeader.A, {
              className: transfer_list_classes.cardHeader,
              avatar: react.createElement(Checkbox.A, {
                onClick: handleToggleAllClick,
                checked: numberOfChecked > 0,
                indeterminate: numberOfChecked > 0 && numberOfChecked < items.length,
                disabled: 0 === items.length,
                inputProps: { 'aria-label': 'all items selected' },
              }),
              title,
              subheader: ''.concat(numberOfChecked, '/').concat(items.length, ' selected'),
            }),
            react.createElement(Divider.A, null),
            react.createElement(
              List.A,
              {
                className: transfer_list_classes.list,
                dense: !0,
                disablePadding: !0,
                component: 'div',
                role: 'list',
              },
              items.map(function (item) {
                var labelId = 'transfer-list-all-item-'.concat(item, '-label');
                return react.createElement(
                  ListItem.Ay,
                  {
                    key: item,
                    role: 'listitem',
                    button: !0,
                    onClick: function () {
                      return setChecked(function (prev) {
                        return prev.has(item) ? prev.delete(item) : prev.add(item), new Set(prev);
                      });
                    },
                  },
                  react.createElement(
                    ListItemIcon.A,
                    null,
                    react.createElement(Checkbox.A, {
                      checked: checked.has(item),
                      tabIndex: -1,
                      disableRipple: !0,
                      inputProps: { 'aria-labelledby': labelId },
                    }),
                  ),
                  react.createElement(ListItemText.A, { id: labelId, primary: item }),
                );
              }),
              react.createElement(ListItem.Ay, null),
            ),
          ),
        );
      }
      function TransferList(_a) {
        var leftItems = _a.leftItems,
          rightItems = _a.rightItems,
          _b = _a.leftTitle,
          leftTitle = void 0 === _b ? 'Choices' : _b,
          _c = _a.rightTitle,
          rightTitle = void 0 === _c ? 'Choices' : _c,
          onTransfer = _a.onTransfer,
          _d = react.useState(new Set()),
          leftChecked = _d[0],
          setLeftChecked = _d[1],
          _e = react.useState(new Set()),
          rightChecked = _e[0],
          setRightChecked = _e[1];
        return react.createElement(
          StyledGrid,
          {
            container: !0,
            spacing: 2,
            justifyContent: 'center',
            alignItems: 'stretch',
            className: transfer_list_classes.container,
          },
          react.createElement(
            Grid.Ay,
            { item: !0 },
            react.createElement(CustomList, {
              title: leftTitle,
              items: leftItems,
              checked: leftChecked,
              setChecked: setLeftChecked,
            }),
          ),
          react.createElement(
            Grid.Ay,
            { item: !0, className: transfer_list_classes.transferControls },
            react.createElement(
              Grid.Ay,
              { container: !0, direction: 'column', justifyContent: 'center' },
              react.createElement(
                Grid.Ay,
                { item: !0 },
                react.createElement(
                  Button.A,
                  {
                    variant: 'outlined',
                    size: 'small',
                    className: transfer_list_classes.button,
                    onClick: function () {
                      var newLeft = leftItems.filter(function (val) {
                          return !leftChecked.has(val);
                        }),
                        newRight = rightItems.concat(Array.from(leftChecked.values()));
                      onTransfer && onTransfer(newLeft, newRight), setLeftChecked(new Set());
                    },
                    disabled: 0 === leftChecked.size,
                    'aria-label': 'move selected right',
                  },
                  '>',
                ),
              ),
              react.createElement(
                Grid.Ay,
                { item: !0 },
                react.createElement(
                  Button.A,
                  {
                    variant: 'outlined',
                    size: 'small',
                    className: transfer_list_classes.button,
                    onClick: function () {
                      var newRight = rightItems.filter(function (val) {
                          return !rightChecked.has(val);
                        }),
                        newLeft = leftItems.concat(Array.from(rightChecked.values()));
                      onTransfer && onTransfer(newLeft, newRight), setRightChecked(new Set());
                    },
                    disabled: 0 === rightChecked.size,
                    'aria-label': 'move selected left',
                  },
                  '<',
                ),
              ),
            ),
          ),
          react.createElement(
            Grid.Ay,
            { item: !0 },
            react.createElement(CustomList, {
              title: rightTitle,
              items: rightItems,
              checked: rightChecked,
              setChecked: setRightChecked,
            }),
          ),
        );
      }
      TransferList.__docgenInfo = { description: '', methods: [], displayName: 'TransferList' };
      var ComponentUnmountedError = 'ComponentUnmountedError';
      function useAsync(throwOnUnmounted) {
        void 0 === throwOnUnmounted && (throwOnUnmounted = !1);
        var mountedRef = react.useRef(!0);
        return (
          react.useEffect(function () {
            return function () {
              mountedRef.current = !1;
            };
          }, []),
          react.useCallback(
            function (maybePromise) {
              return maybePromise instanceof Promise
                ? new Promise(function (res, rej) {
                    maybePromise
                      .then(function (result) {
                        if (mountedRef.current) res(result);
                        else if (throwOnUnmounted) {
                          var error = new Error('component is unmounted or promise was cancelled');
                          throw ((error.name = ComponentUnmountedError), error);
                        }
                      })
                      .catch(rej);
                  })
                : Promise.resolve(maybePromise);
            },
            [throwOnUnmounted],
          )
        );
      }
      var Close = __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/Close.js',
        ),
        WindowManagerStateContext =
          (__webpack_require__(
            '../../node_modules/.pnpm/react-grid-layout@1.3.4_react-dom@18.2.0_react@18.2.0/node_modules/react-grid-layout/css/styles.css',
          ),
          react.createContext({ designMode: !1 })),
        Toolbar = __webpack_require__(
          '../../node_modules/.pnpm/@mui+material@5.8.7_@emotion+react@11.9.3_@emotion+styled@11.9.3_@types+react@18.2.14_react-dom@18.2.0_react@18.2.0/node_modules/@mui/material/Toolbar/Toolbar.js',
        ),
        window_toolbar_assign = function () {
          return (
            (window_toolbar_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            window_toolbar_assign.apply(this, arguments)
          );
        },
        window_toolbar_rest = function (s, e) {
          var t = {};
          for (var p in s)
            Object.prototype.hasOwnProperty.call(s, p) && e.indexOf(p) < 0 && (t[p] = s[p]);
          if (null != s && 'function' == typeof Object.getOwnPropertySymbols) {
            var i = 0;
            for (p = Object.getOwnPropertySymbols(s); i < p.length; i++)
              e.indexOf(p[i]) < 0 &&
                Object.prototype.propertyIsEnumerable.call(s, p[i]) &&
                (t[p[i]] = s[p[i]]);
          }
          return t;
        },
        WindowToolbar = function (_a) {
          var title = _a.title,
            children = _a.children,
            otherProps = window_toolbar_rest(_a, ['title', 'children']);
          return react.createElement(
            AppBar.A,
            window_toolbar_assign({ position: 'static', elevation: 0 }, otherProps),
            react.createElement(
              Toolbar.A,
              { variant: 'dense', style: { paddingRight: 0 } },
              react.createElement(Typography.A, { variant: 'h6', style: { flexGrow: 1 } }, title),
              children,
            ),
          );
        };
      WindowToolbar.__docgenInfo = { description: '', methods: [], displayName: 'WindowToolbar' };
      var window_assign = function () {
          return (
            (window_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            window_assign.apply(this, arguments)
          );
        },
        window_rest = function (s, e) {
          var t = {};
          for (var p in s)
            Object.prototype.hasOwnProperty.call(s, p) && e.indexOf(p) < 0 && (t[p] = s[p]);
          if (null != s && 'function' == typeof Object.getOwnPropertySymbols) {
            var i = 0;
            for (p = Object.getOwnPropertySymbols(s); i < p.length; i++)
              e.indexOf(p[i]) < 0 &&
                Object.prototype.propertyIsEnumerable.call(s, p[i]) &&
                (t[p[i]] = s[p[i]]);
          }
          return t;
        },
        react_grid_layout =
          ((0, styled.Ay)(
            react.forwardRef(function (_a, ref) {
              var title = _a.title,
                toolbar = _a.toolbar,
                onClose = _a.onClose,
                sx = _a.sx,
                children = _a.children,
                otherProps = window_rest(_a, ['title', 'toolbar', 'onClose', 'sx', 'children']),
                theme = (0, useTheme.A)(),
                childrenArr = react.Children.toArray(children),
                childComponents = childrenArr.slice(0, childrenArr.length - 1),
                resizeComponent = childrenArr[childrenArr.length - 1],
                windowManagerState = react.useContext(WindowManagerStateContext);
              return react.createElement(
                Paper.A,
                window_assign(
                  {
                    ref,
                    variant: 'outlined',
                    sx: window_assign(
                      {
                        cursor: windowManagerState.designMode ? 'move' : void 0,
                        borderRadius: theme.shape.borderRadius,
                      },
                      sx,
                    ),
                  },
                  otherProps,
                ),
                react.createElement(
                  Grid.Ay,
                  { item: !0, className: 'rgl-draggable' },
                  react.createElement(
                    WindowToolbar,
                    { title },
                    toolbar,
                    windowManagerState.designMode &&
                      react.createElement(
                        IconButton.A,
                        {
                          color: 'inherit',
                          onClick: function () {
                            return onClose && onClose();
                          },
                        },
                        react.createElement(Close.A, null),
                      ),
                  ),
                ),
                react.createElement(
                  'div',
                  { style: { overflow: 'auto', width: '100%', height: '100%', cursor: 'auto' } },
                  childComponents,
                ),
                resizeComponent,
              );
            }),
          )({ display: 'flex', flexDirection: 'column', flexWrap: 'nowrap', overflow: 'hidden' }),
          __webpack_require__(
            '../../node_modules/.pnpm/react-grid-layout@1.3.4_react-dom@18.2.0_react@18.2.0/node_modules/react-grid-layout/index.js',
          )),
        react_grid_layout_default = __webpack_require__.n(react_grid_layout);
      (0, react_grid_layout.WidthProvider)(react_grid_layout_default()),
        __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/ViewList.js',
        ),
        __webpack_require__(
          '../../node_modules/.pnpm/@mui+icons-material@5.8.4_@mui+material@5.8.7_@types+react@18.2.14_react@18.2.0/node_modules/@mui/icons-material/ViewModule.js',
        ),
        __webpack_require__(
          '../../node_modules/.pnpm/react-virtualized-auto-sizer@1.0.24_react-dom@18.2.0_react@18.2.0/node_modules/react-virtualized-auto-sizer/dist/react-virtualized-auto-sizer.esm.js',
        ),
        __webpack_require__(
          '../../node_modules/.pnpm/react-window@1.8.7_react-dom@18.2.0_react@18.2.0/node_modules/react-window/dist/index.esm.js',
        );
      var workcell_table_classes_dispenserLabelIdle = 'workcell-dispenser-label-idle',
        workcell_table_classes_dispenserLabelBusy = 'workcell-dispenser-label-busy',
        workcell_table_classes_dispenserLabelOffline = 'workcell-offline-label',
        workcell_table_classes_tableRow = 'workcell-table-row',
        workcell_table_classes_tableCell = 'workcell-table-cell',
        WorkcellRow = react.memo(function (_a) {
          var workcell = _a.workcell,
            mode = _a.mode,
            requestGuidQueue = _a.requestGuidQueue,
            secondsRemaining = _a.secondsRemaining,
            fixedTableCell = useFixedTableCellStylesClasses.fixedTableCell,
            dispenserModeLabelClasses = react.useCallback(function (mode) {
              switch (mode) {
                case dist.DispenserState.IDLE:
                  return ''.concat(workcell_table_classes_dispenserLabelIdle);
                case dist.DispenserState.BUSY:
                  return ''.concat(workcell_table_classes_dispenserLabelBusy);
                case dist.DispenserState.OFFLINE:
                  return ''.concat(workcell_table_classes_dispenserLabelOffline);
                default:
                  return '';
              }
            }, []);
          return react.createElement(
            TableRow.A,
            {
              'aria-label': ''.concat(workcell.guid),
              className: workcell_table_classes_tableRow,
              component: 'div',
            },
            void 0 !== mode && void 0 !== requestGuidQueue && void 0 !== secondsRemaining
              ? react.createElement(
                  react.Fragment,
                  null,
                  react.createElement(
                    ItemTableCell,
                    {
                      component: 'div',
                      variant: 'body',
                      className: (0, clsx_m.default)(
                        workcell_table_classes_tableCell,
                        fixedTableCell,
                      ),
                      title: workcell.guid,
                    },
                    workcell.guid,
                  ),
                  react.createElement(
                    ItemTableCell,
                    {
                      component: 'div',
                      variant: 'body',
                      className: (0, clsx_m.default)(
                        dispenserModeLabelClasses(mode),
                        workcell_table_classes_tableCell,
                        fixedTableCell,
                      ),
                    },
                    (function dispenserModeToString(mode) {
                      switch (mode) {
                        case dist.DispenserState.IDLE:
                          return 'IDLE';
                        case dist.DispenserState.BUSY:
                          return 'ONLINE';
                        case dist.DispenserState.OFFLINE:
                          return 'OFFLINE';
                        default:
                          return 'N/A';
                      }
                    })(mode),
                  ),
                  react.createElement(
                    ItemTableCell,
                    {
                      component: 'div',
                      variant: 'body',
                      className: (0, clsx_m.default)(
                        workcell_table_classes_tableCell,
                        fixedTableCell,
                      ),
                    },
                    requestGuidQueue.length,
                  ),
                  react.createElement(
                    ItemTableCell,
                    {
                      component: 'div',
                      variant: 'body',
                      className: (0, clsx_m.default)(
                        workcell_table_classes_tableCell,
                        fixedTableCell,
                      ),
                    },
                    requestGuidQueue,
                  ),
                  react.createElement(
                    ItemTableCell,
                    {
                      component: 'div',
                      variant: 'body',
                      className: (0, clsx_m.default)(
                        workcell_table_classes_tableCell,
                        fixedTableCell,
                      ),
                    },
                    secondsRemaining,
                  ),
                )
              : react.createElement(
                  react.Fragment,
                  null,
                  react.createElement(
                    ItemTableCell,
                    {
                      component: 'div',
                      variant: 'body',
                      className: (0, clsx_m.default)(
                        workcell_table_classes_tableCell,
                        fixedTableCell,
                      ),
                      title: workcell.guid,
                    },
                    workcell.guid,
                  ),
                  react.createElement(
                    ItemTableCell,
                    {
                      component: 'div',
                      variant: 'body',
                      className: (0, clsx_m.default)(
                        workcell_table_classes_tableCell,
                        fixedTableCell,
                      ),
                    },
                    'NA',
                  ),
                  react.createElement(
                    ItemTableCell,
                    {
                      component: 'div',
                      variant: 'body',
                      className: (0, clsx_m.default)(
                        workcell_table_classes_tableCell,
                        fixedTableCell,
                      ),
                    },
                    'NA',
                  ),
                  react.createElement(
                    ItemTableCell,
                    {
                      component: 'div',
                      variant: 'body',
                      className: (0, clsx_m.default)(
                        workcell_table_classes_tableCell,
                        fixedTableCell,
                      ),
                    },
                    'NA',
                  ),
                  react.createElement(
                    ItemTableCell,
                    {
                      component: 'div',
                      variant: 'body',
                      className: (0, clsx_m.default)(
                        workcell_table_classes_tableCell,
                        fixedTableCell,
                      ),
                    },
                    'NA',
                  ),
                ),
          );
        }),
        WorkcellTable = function (_a) {
          var workcells = _a.workcells,
            workcellStates = _a.workcellStates;
          return react.createElement(
            Table.A,
            { component: 'div', size: 'small', 'aria-label': 'workcell-table' },
            react.createElement(
              TableHead.A,
              { component: 'div' },
              react.createElement(
                TableRow.A,
                { component: 'div', className: workcell_table_classes_tableRow },
                react.createElement(
                  ItemTableCell,
                  { component: 'div', variant: 'head' },
                  'Dispenser Name',
                ),
                react.createElement(
                  ItemTableCell,
                  { component: 'div', variant: 'head' },
                  'Op. Mode',
                ),
                react.createElement(
                  ItemTableCell,
                  { component: 'div', variant: 'head' },
                  'No. Queued Requests',
                ),
                react.createElement(
                  ItemTableCell,
                  { component: 'div', variant: 'head' },
                  'Request Queue ID',
                ),
                react.createElement(
                  ItemTableCell,
                  { component: 'div', variant: 'head' },
                  'Seconds Remaining',
                ),
              ),
            ),
            react.createElement(
              TableBody.A,
              { component: 'div' },
              workcells.map(function (workcell) {
                var workcellState = workcellStates[workcell.guid];
                return react.createElement(WorkcellRow, {
                  workcell,
                  mode: null == workcellState ? void 0 : workcellState.mode,
                  requestGuidQueue:
                    null == workcellState ? void 0 : workcellState.request_guid_queue,
                  secondsRemaining:
                    null == workcellState ? void 0 : workcellState.seconds_remaining,
                });
              }),
            ),
          );
        };
      WorkcellTable.__docgenInfo = { description: '', methods: [], displayName: 'WorkcellTable' };
      var workcell_panel_assign = function () {
          return (
            (workcell_panel_assign =
              Object.assign ||
              function (t) {
                for (var s, i = 1, n = arguments.length; i < n; i++)
                  for (var p in (s = arguments[i]))
                    Object.prototype.hasOwnProperty.call(s, p) && (t[p] = s[p]);
                return t;
              }),
            workcell_panel_assign.apply(this, arguments)
          );
        },
        workcell_panel_classes_container = 'workcell-panel-container',
        workcell_panel_classes_buttonBar = 'workcell-buttonbar',
        workcell_panel_classes_cellContainer = 'workcell-cell-container',
        workcell_panel_classes_cellPaper = 'workcell-cell-paper',
        workcell_panel_classes_itemIcon = 'workcell-item-icon',
        workcell_panel_classes_panelHeader = 'workcell-panel-header',
        workcell_panel_classes_subPanelHeader = 'workcell-sub-panel-header',
        workcell_panel_classes_tableDiv = 'workcell-table-div',
        workcell_panel_classes_nameField = 'workcell-name-field',
        workcell_panel_classes_grid = 'workcell-grid';
      (0, styled.Ay)(function (props) {
        return react.createElement(Card.A, workcell_panel_assign({}, props));
      })(function (_a) {
        var _b,
          theme = _a.theme;
        return (
          ((_b = {})['&.'.concat(workcell_panel_classes_container)] = { margin: theme.spacing(1) }),
          (_b['& .'.concat(workcell_panel_classes_buttonBar)] = {
            display: 'flex',
            justifyContent: 'flex-end',
            borderRadius: 0,
            backgroundColor: theme.palette.primary.main,
          }),
          (_b['& .'.concat(workcell_panel_classes_cellContainer)] = {
            padding: theme.spacing(1),
            maxHeight: '25vh',
            margin: theme.spacing(1),
            overflowY: 'auto',
            overflowX: 'hidden',
          }),
          (_b['& .'.concat(workcell_panel_classes_cellPaper)] = {
            padding: theme.spacing(1),
            backgroundColor: theme.palette.background.paper,
            border: 1,
            borderStyle: 'solid',
            borderColor: theme.palette.primary.main,
            '&:hover': { cursor: 'pointer', backgroundColor: theme.palette.action.hover },
            margin: theme.spacing(1),
            height: '60%',
          }),
          (_b['& .'.concat(workcell_panel_classes_grid)] = {
            padding: theme.spacing(2),
            paddingTop: theme.spacing(1),
          }),
          (_b['& .'.concat(workcell_panel_classes_itemIcon)] = {
            color: theme.palette.primary.contrastText,
          }),
          (_b['& .'.concat(workcell_panel_classes_panelHeader)] = {
            color: theme.palette.primary.contrastText,
            marginLeft: theme.spacing(2),
          }),
          (_b['& .'.concat(workcell_panel_classes_subPanelHeader)] = {
            marginLeft: theme.spacing(2),
            color: theme.palette.primary.contrastText,
          }),
          (_b['& .'.concat(workcell_panel_classes_tableDiv)] = {
            margin: theme.spacing(1),
            padding: theme.spacing(1),
          }),
          (_b['& .'.concat(workcell_panel_classes_nameField)] = {
            fontWeight: 'bold',
            whiteSpace: 'nowrap',
            overflow: 'hidden',
            textOverflow: 'ellipsis',
          }),
          (_b.bottomTable = { marginTop: theme.spacing(2) }),
          _b
        );
      });
    },
    '../rmf-models/dist/index.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __createBinding =
          (this && this.__createBinding) ||
          (Object.create
            ? function (o, m, k, k2) {
                void 0 === k2 && (k2 = k),
                  Object.defineProperty(o, k2, {
                    enumerable: !0,
                    get: function () {
                      return m[k];
                    },
                  });
              }
            : function (o, m, k, k2) {
                void 0 === k2 && (k2 = k), (o[k2] = m[k]);
              }),
        __exportStar =
          (this && this.__exportStar) ||
          function (m, exports) {
            for (var p in m)
              'default' === p ||
                Object.prototype.hasOwnProperty.call(exports, p) ||
                __createBinding(exports, m, p);
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        __exportStar(__webpack_require__('../rmf-models/dist/ros/index.js'), exports),
        __exportStar(__webpack_require__('../rmf-models/dist/version.js'), exports);
    },
    '../rmf-models/dist/ros/builtin_interfaces/msg/Duration.js': (
      __unused_webpack_module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.Duration = void 0);
      var Duration = (function () {
        function Duration(fields) {
          void 0 === fields && (fields = {}),
            (this.sec = fields.sec || 0),
            (this.nanosec = fields.nanosec || 0);
        }
        return (
          (Duration.validate = function (obj) {
            if ('number' != typeof obj.sec) throw new Error('expected "sec" to be "number"');
            if ('number' != typeof obj.nanosec)
              throw new Error('expected "nanosec" to be "number"');
          }),
          (Duration.FullTypeName = 'builtin_interfaces/msg/Duration'),
          Duration
        );
      })();
      exports.Duration = Duration;
    },
    '../rmf-models/dist/ros/builtin_interfaces/msg/Time.js': (__unused_webpack_module, exports) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.Time = void 0);
      var Time = (function () {
        function Time(fields) {
          void 0 === fields && (fields = {}),
            (this.sec = fields.sec || 0),
            (this.nanosec = fields.nanosec || 0);
        }
        return (
          (Time.validate = function (obj) {
            if ('number' != typeof obj.sec) throw new Error('expected "sec" to be "number"');
            if ('number' != typeof obj.nanosec)
              throw new Error('expected "nanosec" to be "number"');
          }),
          (Time.FullTypeName = 'builtin_interfaces/msg/Time'),
          Time
        );
      })();
      exports.Time = Time;
    },
    '../rmf-models/dist/ros/index.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __createBinding =
          (this && this.__createBinding) ||
          (Object.create
            ? function (o, m, k, k2) {
                void 0 === k2 && (k2 = k),
                  Object.defineProperty(o, k2, {
                    enumerable: !0,
                    get: function () {
                      return m[k];
                    },
                  });
              }
            : function (o, m, k, k2) {
                void 0 === k2 && (k2 = k), (o[k2] = m[k]);
              }),
        __exportStar =
          (this && this.__exportStar) ||
          function (m, exports) {
            for (var p in m)
              'default' === p ||
                Object.prototype.hasOwnProperty.call(exports, p) ||
                __createBinding(exports, m, p);
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_building_map_msgs/msg/AffineImage.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_building_map_msgs/msg/BuildingMap.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_building_map_msgs/msg/Door.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_building_map_msgs/msg/Graph.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_building_map_msgs/msg/GraphEdge.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_building_map_msgs/msg/GraphNode.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_building_map_msgs/msg/Level.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_building_map_msgs/msg/Lift.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_building_map_msgs/msg/Param.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_building_map_msgs/msg/Place.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__(
            '../rmf-models/dist/ros/rmf_building_map_msgs/srv/GetBuildingMap_Request.js',
          ),
          exports,
        ),
        __exportStar(
          __webpack_require__(
            '../rmf-models/dist/ros/rmf_building_map_msgs/srv/GetBuildingMap_Response.js',
          ),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/builtin_interfaces/msg/Duration.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/builtin_interfaces/msg/Time.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_charger_msgs/msg/ChargerCancel.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_charger_msgs/msg/ChargerRequest.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_charger_msgs/msg/ChargerState.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_door_msgs/msg/DoorMode.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_door_msgs/msg/DoorRequest.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_door_msgs/msg/DoorSessions.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_door_msgs/msg/DoorState.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_door_msgs/msg/Session.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_door_msgs/msg/SupervisorHeartbeat.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_lift_msgs/msg/LiftRequest.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_lift_msgs/msg/LiftState.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_dispenser_msgs/msg/DispenserRequest.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__(
            '../rmf-models/dist/ros/rmf_dispenser_msgs/msg/DispenserRequestItem.js',
          ),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_dispenser_msgs/msg/DispenserResult.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_dispenser_msgs/msg/DispenserState.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_ingestor_msgs/msg/IngestorRequest.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__(
            '../rmf-models/dist/ros/rmf_ingestor_msgs/msg/IngestorRequestItem.js',
          ),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_ingestor_msgs/msg/IngestorResult.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_ingestor_msgs/msg/IngestorState.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_fleet_msgs/msg/ClosedLanes.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_fleet_msgs/msg/DestinationRequest.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_fleet_msgs/msg/Dock.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_fleet_msgs/msg/DockParameter.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_fleet_msgs/msg/DockSummary.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_fleet_msgs/msg/FleetState.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_fleet_msgs/msg/LaneRequest.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_fleet_msgs/msg/Location.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_fleet_msgs/msg/ModeParameter.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_fleet_msgs/msg/ModeRequest.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_fleet_msgs/msg/PathRequest.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_fleet_msgs/msg/PauseRequest.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_fleet_msgs/msg/RobotMode.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_fleet_msgs/msg/RobotState.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_fleet_msgs/srv/LiftClearance_Request.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__(
            '../rmf-models/dist/ros/rmf_fleet_msgs/srv/LiftClearance_Response.js',
          ),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/Behavior.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/BehaviorParameter.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/BidNotice.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/BidProposal.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/Clean.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/Delivery.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/DispatchAck.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/DispatchRequest.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/Loop.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/Priority.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/Station.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/TaskDescription.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/TaskProfile.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/TaskSummary.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/TaskType.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/Tasks.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/Tow.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/srv/CancelTask_Request.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/srv/CancelTask_Response.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/srv/GetTaskList_Request.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/srv/GetTaskList_Response.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/srv/ReviveTask_Request.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/srv/ReviveTask_Response.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/srv/SubmitTask_Request.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/srv/SubmitTask_Response.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_building_map_msgs/srv/GetBuildingMap.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_fleet_msgs/srv/LiftClearance.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/srv/CancelTask.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/srv/GetTaskList.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/srv/ReviveTask.js'),
          exports,
        ),
        __exportStar(
          __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/srv/SubmitTask.js'),
          exports,
        );
    },
    '../rmf-models/dist/ros/rmf_building_map_msgs/msg/AffineImage.js': function (
      __unused_webpack_module,
      exports,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.AffineImage = void 0);
      var AffineImage = (function () {
        function AffineImage(fields) {
          void 0 === fields && (fields = {}),
            (this.name = fields.name || ''),
            (this.x_offset = fields.x_offset || 0),
            (this.y_offset = fields.y_offset || 0),
            (this.yaw = fields.yaw || 0),
            (this.scale = fields.scale || 0),
            (this.encoding = fields.encoding || ''),
            (this.data = fields.data || []);
        }
        return (
          (AffineImage.validate = function (obj) {
            var e_1, _a;
            if ('string' != typeof obj.name) throw new Error('expected "name" to be "string"');
            if ('number' != typeof obj.x_offset)
              throw new Error('expected "x_offset" to be "number"');
            if ('number' != typeof obj.y_offset)
              throw new Error('expected "y_offset" to be "number"');
            if ('number' != typeof obj.yaw) throw new Error('expected "yaw" to be "number"');
            if ('number' != typeof obj.scale) throw new Error('expected "scale" to be "number"');
            if ('string' != typeof obj.encoding)
              throw new Error('expected "encoding" to be "string"');
            if (!(obj.data instanceof Uint8Array || Array.isArray(obj.data)))
              throw new Error('expected "data" to be "Uint8Array" or an array');
            if (Array.isArray(obj.data))
              try {
                for (
                  var _b = __values(obj.data.entries()), _c = _b.next();
                  !_c.done;
                  _c = _b.next()
                ) {
                  var _d = __read(_c.value, 2),
                    i = _d[0];
                  if ('number' != typeof _d[1])
                    throw new Error('expected index ' + i + ' of "data" to be "number"');
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _c && !_c.done && (_a = _b.return) && _a.call(_b);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
          }),
          (AffineImage.FullTypeName = 'rmf_building_map_msgs/msg/AffineImage'),
          AffineImage
        );
      })();
      exports.AffineImage = AffineImage;
    },
    '../rmf-models/dist/ros/rmf_building_map_msgs/msg/BuildingMap.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.BuildingMap = void 0);
      var Level_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_building_map_msgs/msg/Level.js',
        ),
        Lift_1 = __webpack_require__('../rmf-models/dist/ros/rmf_building_map_msgs/msg/Lift.js'),
        BuildingMap = (function () {
          function BuildingMap(fields) {
            void 0 === fields && (fields = {}),
              (this.name = fields.name || ''),
              (this.levels = fields.levels || []),
              (this.lifts = fields.lifts || []);
          }
          return (
            (BuildingMap.validate = function (obj) {
              var e_1, _a, e_2, _b;
              if ('string' != typeof obj.name) throw new Error('expected "name" to be "string"');
              if (!Array.isArray(obj.levels)) throw new Error('expected "levels" to be an array');
              try {
                for (
                  var _c = __values(obj.levels.entries()), _d = _c.next();
                  !_d.done;
                  _d = _c.next()
                ) {
                  var _e = __read(_d.value, 2),
                    i = _e[0],
                    v = _e[1];
                  try {
                    Level_1.Level.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "levels":\n  ' + e.message);
                  }
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _d && !_d.done && (_a = _c.return) && _a.call(_c);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
              if (!Array.isArray(obj.lifts)) throw new Error('expected "lifts" to be an array');
              try {
                for (
                  var _f = __values(obj.lifts.entries()), _g = _f.next();
                  !_g.done;
                  _g = _f.next()
                ) {
                  var _h = __read(_g.value, 2);
                  (i = _h[0]), (v = _h[1]);
                  try {
                    Lift_1.Lift.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "lifts":\n  ' + e.message);
                  }
                }
              } catch (e_2_1) {
                e_2 = { error: e_2_1 };
              } finally {
                try {
                  _g && !_g.done && (_b = _f.return) && _b.call(_f);
                } finally {
                  if (e_2) throw e_2.error;
                }
              }
            }),
            (BuildingMap.FullTypeName = 'rmf_building_map_msgs/msg/BuildingMap'),
            BuildingMap
          );
        })();
      exports.BuildingMap = BuildingMap;
    },
    '../rmf-models/dist/ros/rmf_building_map_msgs/msg/Door.js': (
      __unused_webpack_module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.Door = void 0);
      var Door = (function () {
        function Door(fields) {
          void 0 === fields && (fields = {}),
            (this.name = fields.name || ''),
            (this.v1_x = fields.v1_x || 0),
            (this.v1_y = fields.v1_y || 0),
            (this.v2_x = fields.v2_x || 0),
            (this.v2_y = fields.v2_y || 0),
            (this.door_type = fields.door_type || 0),
            (this.motion_range = fields.motion_range || 0),
            (this.motion_direction = fields.motion_direction || 0);
        }
        return (
          (Door.validate = function (obj) {
            if ('string' != typeof obj.name) throw new Error('expected "name" to be "string"');
            if ('number' != typeof obj.v1_x) throw new Error('expected "v1_x" to be "number"');
            if ('number' != typeof obj.v1_y) throw new Error('expected "v1_y" to be "number"');
            if ('number' != typeof obj.v2_x) throw new Error('expected "v2_x" to be "number"');
            if ('number' != typeof obj.v2_y) throw new Error('expected "v2_y" to be "number"');
            if ('number' != typeof obj.door_type)
              throw new Error('expected "door_type" to be "number"');
            if ('number' != typeof obj.motion_range)
              throw new Error('expected "motion_range" to be "number"');
            if ('number' != typeof obj.motion_direction)
              throw new Error('expected "motion_direction" to be "number"');
          }),
          (Door.FullTypeName = 'rmf_building_map_msgs/msg/Door'),
          (Door.DOOR_TYPE_UNDEFINED = 0),
          (Door.DOOR_TYPE_SINGLE_SLIDING = 1),
          (Door.DOOR_TYPE_DOUBLE_SLIDING = 2),
          (Door.DOOR_TYPE_SINGLE_TELESCOPE = 3),
          (Door.DOOR_TYPE_DOUBLE_TELESCOPE = 4),
          (Door.DOOR_TYPE_SINGLE_SWING = 5),
          (Door.DOOR_TYPE_DOUBLE_SWING = 6),
          Door
        );
      })();
      exports.Door = Door;
    },
    '../rmf-models/dist/ros/rmf_building_map_msgs/msg/Graph.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.Graph = void 0);
      var GraphNode_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_building_map_msgs/msg/GraphNode.js',
        ),
        GraphEdge_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_building_map_msgs/msg/GraphEdge.js',
        ),
        Param_1 = __webpack_require__('../rmf-models/dist/ros/rmf_building_map_msgs/msg/Param.js'),
        Graph = (function () {
          function Graph(fields) {
            void 0 === fields && (fields = {}),
              (this.name = fields.name || ''),
              (this.vertices = fields.vertices || []),
              (this.edges = fields.edges || []),
              (this.params = fields.params || []);
          }
          return (
            (Graph.validate = function (obj) {
              var e_1, _a, e_2, _b, e_3, _c;
              if ('string' != typeof obj.name) throw new Error('expected "name" to be "string"');
              if (!Array.isArray(obj.vertices))
                throw new Error('expected "vertices" to be an array');
              try {
                for (
                  var _d = __values(obj.vertices.entries()), _e = _d.next();
                  !_e.done;
                  _e = _d.next()
                ) {
                  var _f = __read(_e.value, 2),
                    i = _f[0],
                    v = _f[1];
                  try {
                    GraphNode_1.GraphNode.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "vertices":\n  ' + e.message);
                  }
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _e && !_e.done && (_a = _d.return) && _a.call(_d);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
              if (!Array.isArray(obj.edges)) throw new Error('expected "edges" to be an array');
              try {
                for (
                  var _g = __values(obj.edges.entries()), _h = _g.next();
                  !_h.done;
                  _h = _g.next()
                ) {
                  var _j = __read(_h.value, 2);
                  (i = _j[0]), (v = _j[1]);
                  try {
                    GraphEdge_1.GraphEdge.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "edges":\n  ' + e.message);
                  }
                }
              } catch (e_2_1) {
                e_2 = { error: e_2_1 };
              } finally {
                try {
                  _h && !_h.done && (_b = _g.return) && _b.call(_g);
                } finally {
                  if (e_2) throw e_2.error;
                }
              }
              if (!Array.isArray(obj.params)) throw new Error('expected "params" to be an array');
              try {
                for (
                  var _k = __values(obj.params.entries()), _l = _k.next();
                  !_l.done;
                  _l = _k.next()
                ) {
                  var _m = __read(_l.value, 2);
                  (i = _m[0]), (v = _m[1]);
                  try {
                    Param_1.Param.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "params":\n  ' + e.message);
                  }
                }
              } catch (e_3_1) {
                e_3 = { error: e_3_1 };
              } finally {
                try {
                  _l && !_l.done && (_c = _k.return) && _c.call(_k);
                } finally {
                  if (e_3) throw e_3.error;
                }
              }
            }),
            (Graph.FullTypeName = 'rmf_building_map_msgs/msg/Graph'),
            Graph
          );
        })();
      exports.Graph = Graph;
    },
    '../rmf-models/dist/ros/rmf_building_map_msgs/msg/GraphEdge.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.GraphEdge = void 0);
      var Param_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_building_map_msgs/msg/Param.js',
        ),
        GraphEdge = (function () {
          function GraphEdge(fields) {
            void 0 === fields && (fields = {}),
              (this.v1_idx = fields.v1_idx || 0),
              (this.v2_idx = fields.v2_idx || 0),
              (this.params = fields.params || []),
              (this.edge_type = fields.edge_type || 0);
          }
          return (
            (GraphEdge.validate = function (obj) {
              var e_1, _a;
              if ('number' != typeof obj.v1_idx)
                throw new Error('expected "v1_idx" to be "number"');
              if ('number' != typeof obj.v2_idx)
                throw new Error('expected "v2_idx" to be "number"');
              if (!Array.isArray(obj.params)) throw new Error('expected "params" to be an array');
              try {
                for (
                  var _b = __values(obj.params.entries()), _c = _b.next();
                  !_c.done;
                  _c = _b.next()
                ) {
                  var _d = __read(_c.value, 2),
                    i = _d[0],
                    v = _d[1];
                  try {
                    Param_1.Param.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "params":\n  ' + e.message);
                  }
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _c && !_c.done && (_a = _b.return) && _a.call(_b);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
              if ('number' != typeof obj.edge_type)
                throw new Error('expected "edge_type" to be "number"');
            }),
            (GraphEdge.FullTypeName = 'rmf_building_map_msgs/msg/GraphEdge'),
            (GraphEdge.EDGE_TYPE_BIDIRECTIONAL = 0),
            (GraphEdge.EDGE_TYPE_UNIDIRECTIONAL = 1),
            GraphEdge
          );
        })();
      exports.GraphEdge = GraphEdge;
    },
    '../rmf-models/dist/ros/rmf_building_map_msgs/msg/GraphNode.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.GraphNode = void 0);
      var Param_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_building_map_msgs/msg/Param.js',
        ),
        GraphNode = (function () {
          function GraphNode(fields) {
            void 0 === fields && (fields = {}),
              (this.x = fields.x || 0),
              (this.y = fields.y || 0),
              (this.name = fields.name || ''),
              (this.params = fields.params || []);
          }
          return (
            (GraphNode.validate = function (obj) {
              var e_1, _a;
              if ('number' != typeof obj.x) throw new Error('expected "x" to be "number"');
              if ('number' != typeof obj.y) throw new Error('expected "y" to be "number"');
              if ('string' != typeof obj.name) throw new Error('expected "name" to be "string"');
              if (!Array.isArray(obj.params)) throw new Error('expected "params" to be an array');
              try {
                for (
                  var _b = __values(obj.params.entries()), _c = _b.next();
                  !_c.done;
                  _c = _b.next()
                ) {
                  var _d = __read(_c.value, 2),
                    i = _d[0],
                    v = _d[1];
                  try {
                    Param_1.Param.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "params":\n  ' + e.message);
                  }
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _c && !_c.done && (_a = _b.return) && _a.call(_b);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
            }),
            (GraphNode.FullTypeName = 'rmf_building_map_msgs/msg/GraphNode'),
            GraphNode
          );
        })();
      exports.GraphNode = GraphNode;
    },
    '../rmf-models/dist/ros/rmf_building_map_msgs/msg/Level.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.Level = void 0);
      var AffineImage_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_building_map_msgs/msg/AffineImage.js',
        ),
        Place_1 = __webpack_require__('../rmf-models/dist/ros/rmf_building_map_msgs/msg/Place.js'),
        Door_1 = __webpack_require__('../rmf-models/dist/ros/rmf_building_map_msgs/msg/Door.js'),
        Graph_1 = __webpack_require__('../rmf-models/dist/ros/rmf_building_map_msgs/msg/Graph.js'),
        Level = (function () {
          function Level(fields) {
            void 0 === fields && (fields = {}),
              (this.name = fields.name || ''),
              (this.elevation = fields.elevation || 0),
              (this.images = fields.images || []),
              (this.places = fields.places || []),
              (this.doors = fields.doors || []),
              (this.nav_graphs = fields.nav_graphs || []),
              (this.wall_graph = fields.wall_graph || new Graph_1.Graph());
          }
          return (
            (Level.validate = function (obj) {
              var e_1, _a, e_2, _b, e_3, _c, e_4, _d;
              if ('string' != typeof obj.name) throw new Error('expected "name" to be "string"');
              if ('number' != typeof obj.elevation)
                throw new Error('expected "elevation" to be "number"');
              if (!Array.isArray(obj.images)) throw new Error('expected "images" to be an array');
              try {
                for (
                  var _e = __values(obj.images.entries()), _f = _e.next();
                  !_f.done;
                  _f = _e.next()
                ) {
                  var _g = __read(_f.value, 2),
                    i = _g[0],
                    v = _g[1];
                  try {
                    AffineImage_1.AffineImage.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "images":\n  ' + e.message);
                  }
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _f && !_f.done && (_a = _e.return) && _a.call(_e);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
              if (!Array.isArray(obj.places)) throw new Error('expected "places" to be an array');
              try {
                for (
                  var _h = __values(obj.places.entries()), _j = _h.next();
                  !_j.done;
                  _j = _h.next()
                ) {
                  var _k = __read(_j.value, 2);
                  (i = _k[0]), (v = _k[1]);
                  try {
                    Place_1.Place.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "places":\n  ' + e.message);
                  }
                }
              } catch (e_2_1) {
                e_2 = { error: e_2_1 };
              } finally {
                try {
                  _j && !_j.done && (_b = _h.return) && _b.call(_h);
                } finally {
                  if (e_2) throw e_2.error;
                }
              }
              if (!Array.isArray(obj.doors)) throw new Error('expected "doors" to be an array');
              try {
                for (
                  var _l = __values(obj.doors.entries()), _m = _l.next();
                  !_m.done;
                  _m = _l.next()
                ) {
                  var _o = __read(_m.value, 2);
                  (i = _o[0]), (v = _o[1]);
                  try {
                    Door_1.Door.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "doors":\n  ' + e.message);
                  }
                }
              } catch (e_3_1) {
                e_3 = { error: e_3_1 };
              } finally {
                try {
                  _m && !_m.done && (_c = _l.return) && _c.call(_l);
                } finally {
                  if (e_3) throw e_3.error;
                }
              }
              if (!Array.isArray(obj.nav_graphs))
                throw new Error('expected "nav_graphs" to be an array');
              try {
                for (
                  var _p = __values(obj.nav_graphs.entries()), _q = _p.next();
                  !_q.done;
                  _q = _p.next()
                ) {
                  var _r = __read(_q.value, 2);
                  (i = _r[0]), (v = _r[1]);
                  try {
                    Graph_1.Graph.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "nav_graphs":\n  ' + e.message);
                  }
                }
              } catch (e_4_1) {
                e_4 = { error: e_4_1 };
              } finally {
                try {
                  _q && !_q.done && (_d = _p.return) && _d.call(_p);
                } finally {
                  if (e_4) throw e_4.error;
                }
              }
              try {
                Graph_1.Graph.validate(obj.wall_graph);
              } catch (e) {
                throw new Error('in "wall_graph":\n  ' + e.message);
              }
            }),
            (Level.FullTypeName = 'rmf_building_map_msgs/msg/Level'),
            Level
          );
        })();
      exports.Level = Level;
    },
    '../rmf-models/dist/ros/rmf_building_map_msgs/msg/Lift.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.Lift = void 0);
      var Door_1 = __webpack_require__('../rmf-models/dist/ros/rmf_building_map_msgs/msg/Door.js'),
        Graph_1 = __webpack_require__('../rmf-models/dist/ros/rmf_building_map_msgs/msg/Graph.js'),
        Lift = (function () {
          function Lift(fields) {
            void 0 === fields && (fields = {}),
              (this.name = fields.name || ''),
              (this.levels = fields.levels || []),
              (this.doors = fields.doors || []),
              (this.wall_graph = fields.wall_graph || new Graph_1.Graph()),
              (this.ref_x = fields.ref_x || 0),
              (this.ref_y = fields.ref_y || 0),
              (this.ref_yaw = fields.ref_yaw || 0),
              (this.width = fields.width || 0),
              (this.depth = fields.depth || 0);
          }
          return (
            (Lift.validate = function (obj) {
              var e_1, _a, e_2, _b;
              if ('string' != typeof obj.name) throw new Error('expected "name" to be "string"');
              if (!Array.isArray(obj.levels)) throw new Error('expected "levels" to be an array');
              try {
                for (
                  var _c = __values(obj.levels.entries()), _d = _c.next();
                  !_d.done;
                  _d = _c.next()
                ) {
                  var _e = __read(_d.value, 2),
                    i = _e[0];
                  if ('string' != typeof (v = _e[1]))
                    throw new Error('expected index ' + i + ' of "levels" to be "string"');
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _d && !_d.done && (_a = _c.return) && _a.call(_c);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
              if (!Array.isArray(obj.doors)) throw new Error('expected "doors" to be an array');
              try {
                for (
                  var _f = __values(obj.doors.entries()), _g = _f.next();
                  !_g.done;
                  _g = _f.next()
                ) {
                  var _h = __read(_g.value, 2),
                    v = ((i = _h[0]), _h[1]);
                  try {
                    Door_1.Door.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "doors":\n  ' + e.message);
                  }
                }
              } catch (e_2_1) {
                e_2 = { error: e_2_1 };
              } finally {
                try {
                  _g && !_g.done && (_b = _f.return) && _b.call(_f);
                } finally {
                  if (e_2) throw e_2.error;
                }
              }
              try {
                Graph_1.Graph.validate(obj.wall_graph);
              } catch (e) {
                throw new Error('in "wall_graph":\n  ' + e.message);
              }
              if ('number' != typeof obj.ref_x) throw new Error('expected "ref_x" to be "number"');
              if ('number' != typeof obj.ref_y) throw new Error('expected "ref_y" to be "number"');
              if ('number' != typeof obj.ref_yaw)
                throw new Error('expected "ref_yaw" to be "number"');
              if ('number' != typeof obj.width) throw new Error('expected "width" to be "number"');
              if ('number' != typeof obj.depth) throw new Error('expected "depth" to be "number"');
            }),
            (Lift.FullTypeName = 'rmf_building_map_msgs/msg/Lift'),
            Lift
          );
        })();
      exports.Lift = Lift;
    },
    '../rmf-models/dist/ros/rmf_building_map_msgs/msg/Param.js': (
      __unused_webpack_module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.Param = void 0);
      var Param = (function () {
        function Param(fields) {
          void 0 === fields && (fields = {}),
            (this.name = fields.name || ''),
            (this.type = fields.type || 0),
            (this.value_int = fields.value_int || 0),
            (this.value_float = fields.value_float || 0),
            (this.value_string = fields.value_string || ''),
            (this.value_bool = fields.value_bool || !1);
        }
        return (
          (Param.validate = function (obj) {
            if ('string' != typeof obj.name) throw new Error('expected "name" to be "string"');
            if ('number' != typeof obj.type) throw new Error('expected "type" to be "number"');
            if ('number' != typeof obj.value_int)
              throw new Error('expected "value_int" to be "number"');
            if ('number' != typeof obj.value_float)
              throw new Error('expected "value_float" to be "number"');
            if ('string' != typeof obj.value_string)
              throw new Error('expected "value_string" to be "string"');
            if ('boolean' != typeof obj.value_bool)
              throw new Error('expected "value_bool" to be "boolean"');
          }),
          (Param.FullTypeName = 'rmf_building_map_msgs/msg/Param'),
          (Param.TYPE_UNDEFINED = 0),
          (Param.TYPE_STRING = 1),
          (Param.TYPE_INT = 2),
          (Param.TYPE_DOUBLE = 3),
          (Param.TYPE_BOOL = 4),
          Param
        );
      })();
      exports.Param = Param;
    },
    '../rmf-models/dist/ros/rmf_building_map_msgs/msg/Place.js': (
      __unused_webpack_module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.Place = void 0);
      var Place = (function () {
        function Place(fields) {
          void 0 === fields && (fields = {}),
            (this.name = fields.name || ''),
            (this.x = fields.x || 0),
            (this.y = fields.y || 0),
            (this.yaw = fields.yaw || 0),
            (this.position_tolerance = fields.position_tolerance || 0),
            (this.yaw_tolerance = fields.yaw_tolerance || 0);
        }
        return (
          (Place.validate = function (obj) {
            if ('string' != typeof obj.name) throw new Error('expected "name" to be "string"');
            if ('number' != typeof obj.x) throw new Error('expected "x" to be "number"');
            if ('number' != typeof obj.y) throw new Error('expected "y" to be "number"');
            if ('number' != typeof obj.yaw) throw new Error('expected "yaw" to be "number"');
            if ('number' != typeof obj.position_tolerance)
              throw new Error('expected "position_tolerance" to be "number"');
            if ('number' != typeof obj.yaw_tolerance)
              throw new Error('expected "yaw_tolerance" to be "number"');
          }),
          (Place.FullTypeName = 'rmf_building_map_msgs/msg/Place'),
          Place
        );
      })();
      exports.Place = Place;
    },
    '../rmf-models/dist/ros/rmf_building_map_msgs/srv/GetBuildingMap.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.GetBuildingMap = void 0);
      var GetBuildingMap_Request_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_building_map_msgs/srv/GetBuildingMap_Request.js',
        ),
        GetBuildingMap_Response_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_building_map_msgs/srv/GetBuildingMap_Response.js',
        ),
        GetBuildingMap = (function () {
          function GetBuildingMap() {}
          return (
            (GetBuildingMap.FullServiceName = 'rmf_building_map_msgs/srv/GetBuildingMap'),
            (GetBuildingMap.Request = GetBuildingMap_Request_1.GetBuildingMap_Request),
            (GetBuildingMap.Response = GetBuildingMap_Response_1.GetBuildingMap_Response),
            GetBuildingMap
          );
        })();
      exports.GetBuildingMap = GetBuildingMap;
    },
    '../rmf-models/dist/ros/rmf_building_map_msgs/srv/GetBuildingMap_Request.js': (
      __unused_webpack_module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.GetBuildingMap_Request = void 0);
      var GetBuildingMap_Request = (function () {
        function GetBuildingMap_Request(fields) {
          void 0 === fields && (fields = {});
        }
        return (
          (GetBuildingMap_Request.validate = function (obj) {}),
          (GetBuildingMap_Request.FullTypeName =
            'rmf_building_map_msgs/srv/GetBuildingMap_Request'),
          GetBuildingMap_Request
        );
      })();
      exports.GetBuildingMap_Request = GetBuildingMap_Request;
    },
    '../rmf-models/dist/ros/rmf_building_map_msgs/srv/GetBuildingMap_Response.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.GetBuildingMap_Response = void 0);
      var BuildingMap_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_building_map_msgs/msg/BuildingMap.js',
        ),
        GetBuildingMap_Response = (function () {
          function GetBuildingMap_Response(fields) {
            void 0 === fields && (fields = {}),
              (this.building_map = fields.building_map || new BuildingMap_1.BuildingMap());
          }
          return (
            (GetBuildingMap_Response.validate = function (obj) {
              try {
                BuildingMap_1.BuildingMap.validate(obj.building_map);
              } catch (e) {
                throw new Error('in "building_map":\n  ' + e.message);
              }
            }),
            (GetBuildingMap_Response.FullTypeName =
              'rmf_building_map_msgs/srv/GetBuildingMap_Response'),
            GetBuildingMap_Response
          );
        })();
      exports.GetBuildingMap_Response = GetBuildingMap_Response;
    },
    '../rmf-models/dist/ros/rmf_charger_msgs/msg/ChargerCancel.js': (
      __unused_webpack_module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.ChargerCancel = void 0);
      var ChargerCancel = (function () {
        function ChargerCancel(fields) {
          void 0 === fields && (fields = {}),
            (this.charger_name = fields.charger_name || ''),
            (this.request_id = fields.request_id || '');
        }
        return (
          (ChargerCancel.validate = function (obj) {
            if ('string' != typeof obj.charger_name)
              throw new Error('expected "charger_name" to be "string"');
            if ('string' != typeof obj.request_id)
              throw new Error('expected "request_id" to be "string"');
          }),
          (ChargerCancel.FullTypeName = 'rmf_charger_msgs/msg/ChargerCancel'),
          ChargerCancel
        );
      })();
      exports.ChargerCancel = ChargerCancel;
    },
    '../rmf-models/dist/ros/rmf_charger_msgs/msg/ChargerRequest.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.ChargerRequest = void 0);
      var Duration_1 = __webpack_require__(
          '../rmf-models/dist/ros/builtin_interfaces/msg/Duration.js',
        ),
        ChargerRequest = (function () {
          function ChargerRequest(fields) {
            void 0 === fields && (fields = {}),
              (this.charger_name = fields.charger_name || ''),
              (this.fleet_name = fields.fleet_name || ''),
              (this.robot_name = fields.robot_name || ''),
              (this.start_timeout = fields.start_timeout || new Duration_1.Duration()),
              (this.request_id = fields.request_id || '');
          }
          return (
            (ChargerRequest.validate = function (obj) {
              if ('string' != typeof obj.charger_name)
                throw new Error('expected "charger_name" to be "string"');
              if ('string' != typeof obj.fleet_name)
                throw new Error('expected "fleet_name" to be "string"');
              if ('string' != typeof obj.robot_name)
                throw new Error('expected "robot_name" to be "string"');
              try {
                Duration_1.Duration.validate(obj.start_timeout);
              } catch (e) {
                throw new Error('in "start_timeout":\n  ' + e.message);
              }
              if ('string' != typeof obj.request_id)
                throw new Error('expected "request_id" to be "string"');
            }),
            (ChargerRequest.FullTypeName = 'rmf_charger_msgs/msg/ChargerRequest'),
            ChargerRequest
          );
        })();
      exports.ChargerRequest = ChargerRequest;
    },
    '../rmf-models/dist/ros/rmf_charger_msgs/msg/ChargerState.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.ChargerState = void 0);
      var Time_1 = __webpack_require__('../rmf-models/dist/ros/builtin_interfaces/msg/Time.js'),
        Duration_1 = __webpack_require__(
          '../rmf-models/dist/ros/builtin_interfaces/msg/Duration.js',
        ),
        ChargerState = (function () {
          function ChargerState(fields) {
            void 0 === fields && (fields = {}),
              (this.charger_time = fields.charger_time || new Time_1.Time()),
              (this.state = fields.state || 0),
              (this.charger_name = fields.charger_name || ''),
              (this.error_message = fields.error_message || ''),
              (this.request_id = fields.request_id || ''),
              (this.robot_fleet = fields.robot_fleet || ''),
              (this.robot_name = fields.robot_name || ''),
              (this.time_to_fully_charged =
                fields.time_to_fully_charged || new Duration_1.Duration());
          }
          return (
            (ChargerState.validate = function (obj) {
              try {
                Time_1.Time.validate(obj.charger_time);
              } catch (e) {
                throw new Error('in "charger_time":\n  ' + e.message);
              }
              if ('number' != typeof obj.state) throw new Error('expected "state" to be "number"');
              if ('string' != typeof obj.charger_name)
                throw new Error('expected "charger_name" to be "string"');
              if ('string' != typeof obj.error_message)
                throw new Error('expected "error_message" to be "string"');
              if ('string' != typeof obj.request_id)
                throw new Error('expected "request_id" to be "string"');
              if ('string' != typeof obj.robot_fleet)
                throw new Error('expected "robot_fleet" to be "string"');
              if ('string' != typeof obj.robot_name)
                throw new Error('expected "robot_name" to be "string"');
              try {
                Duration_1.Duration.validate(obj.time_to_fully_charged);
              } catch (e) {
                throw new Error('in "time_to_fully_charged":\n  ' + e.message);
              }
            }),
            (ChargerState.FullTypeName = 'rmf_charger_msgs/msg/ChargerState'),
            (ChargerState.CHARGER_IDLE = 1),
            (ChargerState.CHARGER_ASSIGNED = 2),
            (ChargerState.CHARGER_CHARGING = 3),
            (ChargerState.CHARGER_RELEASED = 4),
            (ChargerState.CHARGER_ERROR = 200),
            ChargerState
          );
        })();
      exports.ChargerState = ChargerState;
    },
    '../rmf-models/dist/ros/rmf_dispenser_msgs/msg/DispenserRequest.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.DispenserRequest = void 0);
      var Time_1 = __webpack_require__('../rmf-models/dist/ros/builtin_interfaces/msg/Time.js'),
        DispenserRequestItem_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_dispenser_msgs/msg/DispenserRequestItem.js',
        ),
        DispenserRequest = (function () {
          function DispenserRequest(fields) {
            void 0 === fields && (fields = {}),
              (this.time = fields.time || new Time_1.Time()),
              (this.request_guid = fields.request_guid || ''),
              (this.target_guid = fields.target_guid || ''),
              (this.transporter_type = fields.transporter_type || ''),
              (this.items = fields.items || []);
          }
          return (
            (DispenserRequest.validate = function (obj) {
              var e_1, _a;
              try {
                Time_1.Time.validate(obj.time);
              } catch (e) {
                throw new Error('in "time":\n  ' + e.message);
              }
              if ('string' != typeof obj.request_guid)
                throw new Error('expected "request_guid" to be "string"');
              if ('string' != typeof obj.target_guid)
                throw new Error('expected "target_guid" to be "string"');
              if ('string' != typeof obj.transporter_type)
                throw new Error('expected "transporter_type" to be "string"');
              if (!Array.isArray(obj.items)) throw new Error('expected "items" to be an array');
              try {
                for (
                  var _b = __values(obj.items.entries()), _c = _b.next();
                  !_c.done;
                  _c = _b.next()
                ) {
                  var _d = __read(_c.value, 2),
                    i = _d[0],
                    v = _d[1];
                  try {
                    DispenserRequestItem_1.DispenserRequestItem.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "items":\n  ' + e.message);
                  }
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _c && !_c.done && (_a = _b.return) && _a.call(_b);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
            }),
            (DispenserRequest.FullTypeName = 'rmf_dispenser_msgs/msg/DispenserRequest'),
            DispenserRequest
          );
        })();
      exports.DispenserRequest = DispenserRequest;
    },
    '../rmf-models/dist/ros/rmf_dispenser_msgs/msg/DispenserRequestItem.js': (
      __unused_webpack_module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.DispenserRequestItem = void 0);
      var DispenserRequestItem = (function () {
        function DispenserRequestItem(fields) {
          void 0 === fields && (fields = {}),
            (this.type_guid = fields.type_guid || ''),
            (this.quantity = fields.quantity || 0),
            (this.compartment_name = fields.compartment_name || '');
        }
        return (
          (DispenserRequestItem.validate = function (obj) {
            if ('string' != typeof obj.type_guid)
              throw new Error('expected "type_guid" to be "string"');
            if ('number' != typeof obj.quantity)
              throw new Error('expected "quantity" to be "number"');
            if ('string' != typeof obj.compartment_name)
              throw new Error('expected "compartment_name" to be "string"');
          }),
          (DispenserRequestItem.FullTypeName = 'rmf_dispenser_msgs/msg/DispenserRequestItem'),
          DispenserRequestItem
        );
      })();
      exports.DispenserRequestItem = DispenserRequestItem;
    },
    '../rmf-models/dist/ros/rmf_dispenser_msgs/msg/DispenserResult.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.DispenserResult = void 0);
      var Time_1 = __webpack_require__('../rmf-models/dist/ros/builtin_interfaces/msg/Time.js'),
        DispenserResult = (function () {
          function DispenserResult(fields) {
            void 0 === fields && (fields = {}),
              (this.time = fields.time || new Time_1.Time()),
              (this.request_guid = fields.request_guid || ''),
              (this.source_guid = fields.source_guid || ''),
              (this.status = fields.status || 0);
          }
          return (
            (DispenserResult.validate = function (obj) {
              try {
                Time_1.Time.validate(obj.time);
              } catch (e) {
                throw new Error('in "time":\n  ' + e.message);
              }
              if ('string' != typeof obj.request_guid)
                throw new Error('expected "request_guid" to be "string"');
              if ('string' != typeof obj.source_guid)
                throw new Error('expected "source_guid" to be "string"');
              if ('number' != typeof obj.status)
                throw new Error('expected "status" to be "number"');
            }),
            (DispenserResult.FullTypeName = 'rmf_dispenser_msgs/msg/DispenserResult'),
            (DispenserResult.ACKNOWLEDGED = 0),
            (DispenserResult.SUCCESS = 1),
            (DispenserResult.FAILED = 2),
            DispenserResult
          );
        })();
      exports.DispenserResult = DispenserResult;
    },
    '../rmf-models/dist/ros/rmf_dispenser_msgs/msg/DispenserState.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.DispenserState = void 0);
      var Time_1 = __webpack_require__('../rmf-models/dist/ros/builtin_interfaces/msg/Time.js'),
        DispenserState = (function () {
          function DispenserState(fields) {
            void 0 === fields && (fields = {}),
              (this.time = fields.time || new Time_1.Time()),
              (this.guid = fields.guid || ''),
              (this.mode = fields.mode || 0),
              (this.request_guid_queue = fields.request_guid_queue || []),
              (this.seconds_remaining = fields.seconds_remaining || 0);
          }
          return (
            (DispenserState.validate = function (obj) {
              var e_1, _a;
              try {
                Time_1.Time.validate(obj.time);
              } catch (e) {
                throw new Error('in "time":\n  ' + e.message);
              }
              if ('string' != typeof obj.guid) throw new Error('expected "guid" to be "string"');
              if ('number' != typeof obj.mode) throw new Error('expected "mode" to be "number"');
              if (!Array.isArray(obj.request_guid_queue))
                throw new Error('expected "request_guid_queue" to be an array');
              try {
                for (
                  var _b = __values(obj.request_guid_queue.entries()), _c = _b.next();
                  !_c.done;
                  _c = _b.next()
                ) {
                  var _d = __read(_c.value, 2),
                    i = _d[0];
                  if ('string' != typeof _d[1])
                    throw new Error(
                      'expected index ' + i + ' of "request_guid_queue" to be "string"',
                    );
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _c && !_c.done && (_a = _b.return) && _a.call(_b);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
              if ('number' != typeof obj.seconds_remaining)
                throw new Error('expected "seconds_remaining" to be "number"');
            }),
            (DispenserState.FullTypeName = 'rmf_dispenser_msgs/msg/DispenserState'),
            (DispenserState.IDLE = 0),
            (DispenserState.BUSY = 1),
            (DispenserState.OFFLINE = 2),
            DispenserState
          );
        })();
      exports.DispenserState = DispenserState;
    },
    '../rmf-models/dist/ros/rmf_door_msgs/msg/DoorMode.js': (__unused_webpack_module, exports) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.DoorMode = void 0);
      var DoorMode = (function () {
        function DoorMode(fields) {
          void 0 === fields && (fields = {}), (this.value = fields.value || 0);
        }
        return (
          (DoorMode.validate = function (obj) {
            if ('number' != typeof obj.value) throw new Error('expected "value" to be "number"');
          }),
          (DoorMode.FullTypeName = 'rmf_door_msgs/msg/DoorMode'),
          (DoorMode.MODE_CLOSED = 0),
          (DoorMode.MODE_MOVING = 1),
          (DoorMode.MODE_OPEN = 2),
          (DoorMode.MODE_OFFLINE = 3),
          (DoorMode.MODE_UNKNOWN = 4),
          DoorMode
        );
      })();
      exports.DoorMode = DoorMode;
    },
    '../rmf-models/dist/ros/rmf_door_msgs/msg/DoorRequest.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.DoorRequest = void 0);
      var Time_1 = __webpack_require__('../rmf-models/dist/ros/builtin_interfaces/msg/Time.js'),
        DoorMode_1 = __webpack_require__('../rmf-models/dist/ros/rmf_door_msgs/msg/DoorMode.js'),
        DoorRequest = (function () {
          function DoorRequest(fields) {
            void 0 === fields && (fields = {}),
              (this.request_time = fields.request_time || new Time_1.Time()),
              (this.requester_id = fields.requester_id || ''),
              (this.door_name = fields.door_name || ''),
              (this.requested_mode = fields.requested_mode || new DoorMode_1.DoorMode());
          }
          return (
            (DoorRequest.validate = function (obj) {
              try {
                Time_1.Time.validate(obj.request_time);
              } catch (e) {
                throw new Error('in "request_time":\n  ' + e.message);
              }
              if ('string' != typeof obj.requester_id)
                throw new Error('expected "requester_id" to be "string"');
              if ('string' != typeof obj.door_name)
                throw new Error('expected "door_name" to be "string"');
              try {
                DoorMode_1.DoorMode.validate(obj.requested_mode);
              } catch (e) {
                throw new Error('in "requested_mode":\n  ' + e.message);
              }
            }),
            (DoorRequest.FullTypeName = 'rmf_door_msgs/msg/DoorRequest'),
            DoorRequest
          );
        })();
      exports.DoorRequest = DoorRequest;
    },
    '../rmf-models/dist/ros/rmf_door_msgs/msg/DoorSessions.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.DoorSessions = void 0);
      var Session_1 = __webpack_require__('../rmf-models/dist/ros/rmf_door_msgs/msg/Session.js'),
        DoorSessions = (function () {
          function DoorSessions(fields) {
            void 0 === fields && (fields = {}),
              (this.door_name = fields.door_name || ''),
              (this.sessions = fields.sessions || []);
          }
          return (
            (DoorSessions.validate = function (obj) {
              var e_1, _a;
              if ('string' != typeof obj.door_name)
                throw new Error('expected "door_name" to be "string"');
              if (!Array.isArray(obj.sessions))
                throw new Error('expected "sessions" to be an array');
              try {
                for (
                  var _b = __values(obj.sessions.entries()), _c = _b.next();
                  !_c.done;
                  _c = _b.next()
                ) {
                  var _d = __read(_c.value, 2),
                    i = _d[0],
                    v = _d[1];
                  try {
                    Session_1.Session.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "sessions":\n  ' + e.message);
                  }
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _c && !_c.done && (_a = _b.return) && _a.call(_b);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
            }),
            (DoorSessions.FullTypeName = 'rmf_door_msgs/msg/DoorSessions'),
            DoorSessions
          );
        })();
      exports.DoorSessions = DoorSessions;
    },
    '../rmf-models/dist/ros/rmf_door_msgs/msg/DoorState.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.DoorState = void 0);
      var Time_1 = __webpack_require__('../rmf-models/dist/ros/builtin_interfaces/msg/Time.js'),
        DoorMode_1 = __webpack_require__('../rmf-models/dist/ros/rmf_door_msgs/msg/DoorMode.js'),
        DoorState = (function () {
          function DoorState(fields) {
            void 0 === fields && (fields = {}),
              (this.door_time = fields.door_time || new Time_1.Time()),
              (this.door_name = fields.door_name || ''),
              (this.current_mode = fields.current_mode || new DoorMode_1.DoorMode());
          }
          return (
            (DoorState.validate = function (obj) {
              try {
                Time_1.Time.validate(obj.door_time);
              } catch (e) {
                throw new Error('in "door_time":\n  ' + e.message);
              }
              if ('string' != typeof obj.door_name)
                throw new Error('expected "door_name" to be "string"');
              try {
                DoorMode_1.DoorMode.validate(obj.current_mode);
              } catch (e) {
                throw new Error('in "current_mode":\n  ' + e.message);
              }
            }),
            (DoorState.FullTypeName = 'rmf_door_msgs/msg/DoorState'),
            DoorState
          );
        })();
      exports.DoorState = DoorState;
    },
    '../rmf-models/dist/ros/rmf_door_msgs/msg/Session.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.Session = void 0);
      var Time_1 = __webpack_require__('../rmf-models/dist/ros/builtin_interfaces/msg/Time.js'),
        Session = (function () {
          function Session(fields) {
            void 0 === fields && (fields = {}),
              (this.request_time = fields.request_time || new Time_1.Time()),
              (this.requester_id = fields.requester_id || '');
          }
          return (
            (Session.validate = function (obj) {
              try {
                Time_1.Time.validate(obj.request_time);
              } catch (e) {
                throw new Error('in "request_time":\n  ' + e.message);
              }
              if ('string' != typeof obj.requester_id)
                throw new Error('expected "requester_id" to be "string"');
            }),
            (Session.FullTypeName = 'rmf_door_msgs/msg/Session'),
            Session
          );
        })();
      exports.Session = Session;
    },
    '../rmf-models/dist/ros/rmf_door_msgs/msg/SupervisorHeartbeat.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.SupervisorHeartbeat = void 0);
      var DoorSessions_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_door_msgs/msg/DoorSessions.js',
        ),
        SupervisorHeartbeat = (function () {
          function SupervisorHeartbeat(fields) {
            void 0 === fields && (fields = {}), (this.all_sessions = fields.all_sessions || []);
          }
          return (
            (SupervisorHeartbeat.validate = function (obj) {
              var e_1, _a;
              if (!Array.isArray(obj.all_sessions))
                throw new Error('expected "all_sessions" to be an array');
              try {
                for (
                  var _b = __values(obj.all_sessions.entries()), _c = _b.next();
                  !_c.done;
                  _c = _b.next()
                ) {
                  var _d = __read(_c.value, 2),
                    i = _d[0],
                    v = _d[1];
                  try {
                    DoorSessions_1.DoorSessions.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "all_sessions":\n  ' + e.message);
                  }
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _c && !_c.done && (_a = _b.return) && _a.call(_b);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
            }),
            (SupervisorHeartbeat.FullTypeName = 'rmf_door_msgs/msg/SupervisorHeartbeat'),
            SupervisorHeartbeat
          );
        })();
      exports.SupervisorHeartbeat = SupervisorHeartbeat;
    },
    '../rmf-models/dist/ros/rmf_fleet_msgs/msg/ClosedLanes.js': function (
      __unused_webpack_module,
      exports,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.ClosedLanes = void 0);
      var ClosedLanes = (function () {
        function ClosedLanes(fields) {
          void 0 === fields && (fields = {}),
            (this.fleet_name = fields.fleet_name || ''),
            (this.closed_lanes = fields.closed_lanes || []);
        }
        return (
          (ClosedLanes.validate = function (obj) {
            var e_1, _a;
            if ('string' != typeof obj.fleet_name)
              throw new Error('expected "fleet_name" to be "string"');
            if (!(obj.closed_lanes instanceof BigUint64Array || Array.isArray(obj.closed_lanes)))
              throw new Error('expected "closed_lanes" to be "BigUint64Array" or an array');
            if (Array.isArray(obj.closed_lanes))
              try {
                for (
                  var _b = __values(obj.closed_lanes.entries()), _c = _b.next();
                  !_c.done;
                  _c = _b.next()
                ) {
                  var _d = __read(_c.value, 2),
                    i = _d[0];
                  if ('number' != typeof _d[1])
                    throw new Error('expected index ' + i + ' of "closed_lanes" to be "number"');
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _c && !_c.done && (_a = _b.return) && _a.call(_b);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
          }),
          (ClosedLanes.FullTypeName = 'rmf_fleet_msgs/msg/ClosedLanes'),
          ClosedLanes
        );
      })();
      exports.ClosedLanes = ClosedLanes;
    },
    '../rmf-models/dist/ros/rmf_fleet_msgs/msg/DestinationRequest.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.DestinationRequest = void 0);
      var Location_1 = __webpack_require__('../rmf-models/dist/ros/rmf_fleet_msgs/msg/Location.js'),
        DestinationRequest = (function () {
          function DestinationRequest(fields) {
            void 0 === fields && (fields = {}),
              (this.fleet_name = fields.fleet_name || ''),
              (this.robot_name = fields.robot_name || ''),
              (this.destination = fields.destination || new Location_1.Location()),
              (this.task_id = fields.task_id || '');
          }
          return (
            (DestinationRequest.validate = function (obj) {
              if ('string' != typeof obj.fleet_name)
                throw new Error('expected "fleet_name" to be "string"');
              if ('string' != typeof obj.robot_name)
                throw new Error('expected "robot_name" to be "string"');
              try {
                Location_1.Location.validate(obj.destination);
              } catch (e) {
                throw new Error('in "destination":\n  ' + e.message);
              }
              if ('string' != typeof obj.task_id)
                throw new Error('expected "task_id" to be "string"');
            }),
            (DestinationRequest.FullTypeName = 'rmf_fleet_msgs/msg/DestinationRequest'),
            DestinationRequest
          );
        })();
      exports.DestinationRequest = DestinationRequest;
    },
    '../rmf-models/dist/ros/rmf_fleet_msgs/msg/Dock.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.Dock = void 0);
      var DockParameter_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_fleet_msgs/msg/DockParameter.js',
        ),
        Dock = (function () {
          function Dock(fields) {
            void 0 === fields && (fields = {}),
              (this.fleet_name = fields.fleet_name || ''),
              (this.params = fields.params || []);
          }
          return (
            (Dock.validate = function (obj) {
              var e_1, _a;
              if ('string' != typeof obj.fleet_name)
                throw new Error('expected "fleet_name" to be "string"');
              if (!Array.isArray(obj.params)) throw new Error('expected "params" to be an array');
              try {
                for (
                  var _b = __values(obj.params.entries()), _c = _b.next();
                  !_c.done;
                  _c = _b.next()
                ) {
                  var _d = __read(_c.value, 2),
                    i = _d[0],
                    v = _d[1];
                  try {
                    DockParameter_1.DockParameter.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "params":\n  ' + e.message);
                  }
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _c && !_c.done && (_a = _b.return) && _a.call(_b);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
            }),
            (Dock.FullTypeName = 'rmf_fleet_msgs/msg/Dock'),
            Dock
          );
        })();
      exports.Dock = Dock;
    },
    '../rmf-models/dist/ros/rmf_fleet_msgs/msg/DockParameter.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.DockParameter = void 0);
      var Location_1 = __webpack_require__('../rmf-models/dist/ros/rmf_fleet_msgs/msg/Location.js'),
        DockParameter = (function () {
          function DockParameter(fields) {
            void 0 === fields && (fields = {}),
              (this.start = fields.start || ''),
              (this.finish = fields.finish || ''),
              (this.path = fields.path || []);
          }
          return (
            (DockParameter.validate = function (obj) {
              var e_1, _a;
              if ('string' != typeof obj.start) throw new Error('expected "start" to be "string"');
              if ('string' != typeof obj.finish)
                throw new Error('expected "finish" to be "string"');
              if (!Array.isArray(obj.path)) throw new Error('expected "path" to be an array');
              try {
                for (
                  var _b = __values(obj.path.entries()), _c = _b.next();
                  !_c.done;
                  _c = _b.next()
                ) {
                  var _d = __read(_c.value, 2),
                    i = _d[0],
                    v = _d[1];
                  try {
                    Location_1.Location.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "path":\n  ' + e.message);
                  }
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _c && !_c.done && (_a = _b.return) && _a.call(_b);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
            }),
            (DockParameter.FullTypeName = 'rmf_fleet_msgs/msg/DockParameter'),
            DockParameter
          );
        })();
      exports.DockParameter = DockParameter;
    },
    '../rmf-models/dist/ros/rmf_fleet_msgs/msg/DockSummary.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.DockSummary = void 0);
      var Dock_1 = __webpack_require__('../rmf-models/dist/ros/rmf_fleet_msgs/msg/Dock.js'),
        DockSummary = (function () {
          function DockSummary(fields) {
            void 0 === fields && (fields = {}), (this.docks = fields.docks || []);
          }
          return (
            (DockSummary.validate = function (obj) {
              var e_1, _a;
              if (!Array.isArray(obj.docks)) throw new Error('expected "docks" to be an array');
              try {
                for (
                  var _b = __values(obj.docks.entries()), _c = _b.next();
                  !_c.done;
                  _c = _b.next()
                ) {
                  var _d = __read(_c.value, 2),
                    i = _d[0],
                    v = _d[1];
                  try {
                    Dock_1.Dock.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "docks":\n  ' + e.message);
                  }
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _c && !_c.done && (_a = _b.return) && _a.call(_b);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
            }),
            (DockSummary.FullTypeName = 'rmf_fleet_msgs/msg/DockSummary'),
            DockSummary
          );
        })();
      exports.DockSummary = DockSummary;
    },
    '../rmf-models/dist/ros/rmf_fleet_msgs/msg/FleetState.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.FleetState = void 0);
      var RobotState_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_fleet_msgs/msg/RobotState.js',
        ),
        FleetState = (function () {
          function FleetState(fields) {
            void 0 === fields && (fields = {}),
              (this.name = fields.name || ''),
              (this.robots = fields.robots || []);
          }
          return (
            (FleetState.validate = function (obj) {
              var e_1, _a;
              if ('string' != typeof obj.name) throw new Error('expected "name" to be "string"');
              if (!Array.isArray(obj.robots)) throw new Error('expected "robots" to be an array');
              try {
                for (
                  var _b = __values(obj.robots.entries()), _c = _b.next();
                  !_c.done;
                  _c = _b.next()
                ) {
                  var _d = __read(_c.value, 2),
                    i = _d[0],
                    v = _d[1];
                  try {
                    RobotState_1.RobotState.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "robots":\n  ' + e.message);
                  }
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _c && !_c.done && (_a = _b.return) && _a.call(_b);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
            }),
            (FleetState.FullTypeName = 'rmf_fleet_msgs/msg/FleetState'),
            FleetState
          );
        })();
      exports.FleetState = FleetState;
    },
    '../rmf-models/dist/ros/rmf_fleet_msgs/msg/LaneRequest.js': function (
      __unused_webpack_module,
      exports,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.LaneRequest = void 0);
      var LaneRequest = (function () {
        function LaneRequest(fields) {
          void 0 === fields && (fields = {}),
            (this.fleet_name = fields.fleet_name || ''),
            (this.open_lanes = fields.open_lanes || []),
            (this.close_lanes = fields.close_lanes || []);
        }
        return (
          (LaneRequest.validate = function (obj) {
            var e_1, _a, e_2, _b;
            if ('string' != typeof obj.fleet_name)
              throw new Error('expected "fleet_name" to be "string"');
            if (!(obj.open_lanes instanceof BigUint64Array || Array.isArray(obj.open_lanes)))
              throw new Error('expected "open_lanes" to be "BigUint64Array" or an array');
            if (Array.isArray(obj.open_lanes))
              try {
                for (
                  var _c = __values(obj.open_lanes.entries()), _d = _c.next();
                  !_d.done;
                  _d = _c.next()
                ) {
                  var _e = __read(_d.value, 2),
                    i = _e[0];
                  if ('number' != typeof _e[1])
                    throw new Error('expected index ' + i + ' of "open_lanes" to be "number"');
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _d && !_d.done && (_a = _c.return) && _a.call(_c);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
            if (!(obj.close_lanes instanceof BigUint64Array || Array.isArray(obj.close_lanes)))
              throw new Error('expected "close_lanes" to be "BigUint64Array" or an array');
            if (Array.isArray(obj.close_lanes))
              try {
                for (
                  var _f = __values(obj.close_lanes.entries()), _g = _f.next();
                  !_g.done;
                  _g = _f.next()
                ) {
                  var _h = __read(_g.value, 2);
                  i = _h[0];
                  if ('number' != typeof _h[1])
                    throw new Error('expected index ' + i + ' of "close_lanes" to be "number"');
                }
              } catch (e_2_1) {
                e_2 = { error: e_2_1 };
              } finally {
                try {
                  _g && !_g.done && (_b = _f.return) && _b.call(_f);
                } finally {
                  if (e_2) throw e_2.error;
                }
              }
          }),
          (LaneRequest.FullTypeName = 'rmf_fleet_msgs/msg/LaneRequest'),
          LaneRequest
        );
      })();
      exports.LaneRequest = LaneRequest;
    },
    '../rmf-models/dist/ros/rmf_fleet_msgs/msg/Location.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.Location = void 0);
      var Time_1 = __webpack_require__('../rmf-models/dist/ros/builtin_interfaces/msg/Time.js'),
        Location = (function () {
          function Location(fields) {
            void 0 === fields && (fields = {}),
              (this.t = fields.t || new Time_1.Time()),
              (this.x = fields.x || 0),
              (this.y = fields.y || 0),
              (this.yaw = fields.yaw || 0),
              (this.obey_approach_speed_limit = fields.obey_approach_speed_limit || !1),
              (this.approach_speed_limit = fields.approach_speed_limit || 0),
              (this.level_name = fields.level_name || ''),
              (this.index = fields.index || 0);
          }
          return (
            (Location.validate = function (obj) {
              try {
                Time_1.Time.validate(obj.t);
              } catch (e) {
                throw new Error('in "t":\n  ' + e.message);
              }
              if ('number' != typeof obj.x) throw new Error('expected "x" to be "number"');
              if ('number' != typeof obj.y) throw new Error('expected "y" to be "number"');
              if ('number' != typeof obj.yaw) throw new Error('expected "yaw" to be "number"');
              if ('boolean' != typeof obj.obey_approach_speed_limit)
                throw new Error('expected "obey_approach_speed_limit" to be "boolean"');
              if ('number' != typeof obj.approach_speed_limit)
                throw new Error('expected "approach_speed_limit" to be "number"');
              if ('string' != typeof obj.level_name)
                throw new Error('expected "level_name" to be "string"');
              if ('number' != typeof obj.index) throw new Error('expected "index" to be "number"');
            }),
            (Location.FullTypeName = 'rmf_fleet_msgs/msg/Location'),
            Location
          );
        })();
      exports.Location = Location;
    },
    '../rmf-models/dist/ros/rmf_fleet_msgs/msg/ModeParameter.js': (
      __unused_webpack_module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.ModeParameter = void 0);
      var ModeParameter = (function () {
        function ModeParameter(fields) {
          void 0 === fields && (fields = {}),
            (this.name = fields.name || ''),
            (this.value = fields.value || '');
        }
        return (
          (ModeParameter.validate = function (obj) {
            if ('string' != typeof obj.name) throw new Error('expected "name" to be "string"');
            if ('string' != typeof obj.value) throw new Error('expected "value" to be "string"');
          }),
          (ModeParameter.FullTypeName = 'rmf_fleet_msgs/msg/ModeParameter'),
          ModeParameter
        );
      })();
      exports.ModeParameter = ModeParameter;
    },
    '../rmf-models/dist/ros/rmf_fleet_msgs/msg/ModeRequest.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.ModeRequest = void 0);
      var RobotMode_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_fleet_msgs/msg/RobotMode.js',
        ),
        ModeParameter_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_fleet_msgs/msg/ModeParameter.js',
        ),
        ModeRequest = (function () {
          function ModeRequest(fields) {
            void 0 === fields && (fields = {}),
              (this.fleet_name = fields.fleet_name || ''),
              (this.robot_name = fields.robot_name || ''),
              (this.mode = fields.mode || new RobotMode_1.RobotMode()),
              (this.task_id = fields.task_id || ''),
              (this.parameters = fields.parameters || []);
          }
          return (
            (ModeRequest.validate = function (obj) {
              var e_1, _a;
              if ('string' != typeof obj.fleet_name)
                throw new Error('expected "fleet_name" to be "string"');
              if ('string' != typeof obj.robot_name)
                throw new Error('expected "robot_name" to be "string"');
              try {
                RobotMode_1.RobotMode.validate(obj.mode);
              } catch (e) {
                throw new Error('in "mode":\n  ' + e.message);
              }
              if ('string' != typeof obj.task_id)
                throw new Error('expected "task_id" to be "string"');
              if (!Array.isArray(obj.parameters))
                throw new Error('expected "parameters" to be an array');
              try {
                for (
                  var _b = __values(obj.parameters.entries()), _c = _b.next();
                  !_c.done;
                  _c = _b.next()
                ) {
                  var _d = __read(_c.value, 2),
                    i = _d[0],
                    v = _d[1];
                  try {
                    ModeParameter_1.ModeParameter.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "parameters":\n  ' + e.message);
                  }
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _c && !_c.done && (_a = _b.return) && _a.call(_b);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
            }),
            (ModeRequest.FullTypeName = 'rmf_fleet_msgs/msg/ModeRequest'),
            ModeRequest
          );
        })();
      exports.ModeRequest = ModeRequest;
    },
    '../rmf-models/dist/ros/rmf_fleet_msgs/msg/PathRequest.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.PathRequest = void 0);
      var Location_1 = __webpack_require__('../rmf-models/dist/ros/rmf_fleet_msgs/msg/Location.js'),
        PathRequest = (function () {
          function PathRequest(fields) {
            void 0 === fields && (fields = {}),
              (this.fleet_name = fields.fleet_name || ''),
              (this.robot_name = fields.robot_name || ''),
              (this.path = fields.path || []),
              (this.task_id = fields.task_id || '');
          }
          return (
            (PathRequest.validate = function (obj) {
              var e_1, _a;
              if ('string' != typeof obj.fleet_name)
                throw new Error('expected "fleet_name" to be "string"');
              if ('string' != typeof obj.robot_name)
                throw new Error('expected "robot_name" to be "string"');
              if (!Array.isArray(obj.path)) throw new Error('expected "path" to be an array');
              try {
                for (
                  var _b = __values(obj.path.entries()), _c = _b.next();
                  !_c.done;
                  _c = _b.next()
                ) {
                  var _d = __read(_c.value, 2),
                    i = _d[0],
                    v = _d[1];
                  try {
                    Location_1.Location.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "path":\n  ' + e.message);
                  }
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _c && !_c.done && (_a = _b.return) && _a.call(_b);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
              if ('string' != typeof obj.task_id)
                throw new Error('expected "task_id" to be "string"');
            }),
            (PathRequest.FullTypeName = 'rmf_fleet_msgs/msg/PathRequest'),
            PathRequest
          );
        })();
      exports.PathRequest = PathRequest;
    },
    '../rmf-models/dist/ros/rmf_fleet_msgs/msg/PauseRequest.js': (
      __unused_webpack_module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.PauseRequest = void 0);
      var PauseRequest = (function () {
        function PauseRequest(fields) {
          void 0 === fields && (fields = {}),
            (this.fleet_name = fields.fleet_name || ''),
            (this.robot_name = fields.robot_name || ''),
            (this.mode_request_id = fields.mode_request_id || 0),
            (this.type = fields.type || 0),
            (this.at_checkpoint = fields.at_checkpoint || 0);
        }
        return (
          (PauseRequest.validate = function (obj) {
            if ('string' != typeof obj.fleet_name)
              throw new Error('expected "fleet_name" to be "string"');
            if ('string' != typeof obj.robot_name)
              throw new Error('expected "robot_name" to be "string"');
            if ('number' != typeof obj.mode_request_id)
              throw new Error('expected "mode_request_id" to be "number"');
            if ('number' != typeof obj.type) throw new Error('expected "type" to be "number"');
            if ('number' != typeof obj.at_checkpoint)
              throw new Error('expected "at_checkpoint" to be "number"');
          }),
          (PauseRequest.FullTypeName = 'rmf_fleet_msgs/msg/PauseRequest'),
          (PauseRequest.TYPE_PAUSE_IMMEDIATELY = 0),
          (PauseRequest.TYPE_PAUSE_AT_CHECKPOINT = 1),
          (PauseRequest.TYPE_RESUME = 2),
          PauseRequest
        );
      })();
      exports.PauseRequest = PauseRequest;
    },
    '../rmf-models/dist/ros/rmf_fleet_msgs/msg/RobotMode.js': (
      __unused_webpack_module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.RobotMode = void 0);
      var RobotMode = (function () {
        function RobotMode(fields) {
          void 0 === fields && (fields = {}),
            (this.mode = fields.mode || 0),
            (this.mode_request_id = fields.mode_request_id || 0);
        }
        return (
          (RobotMode.validate = function (obj) {
            if ('number' != typeof obj.mode) throw new Error('expected "mode" to be "number"');
            if ('number' != typeof obj.mode_request_id)
              throw new Error('expected "mode_request_id" to be "number"');
          }),
          (RobotMode.FullTypeName = 'rmf_fleet_msgs/msg/RobotMode'),
          (RobotMode.MODE_IDLE = 0),
          (RobotMode.MODE_CHARGING = 1),
          (RobotMode.MODE_MOVING = 2),
          (RobotMode.MODE_PAUSED = 3),
          (RobotMode.MODE_WAITING = 4),
          (RobotMode.MODE_EMERGENCY = 5),
          (RobotMode.MODE_GOING_HOME = 6),
          (RobotMode.MODE_DOCKING = 7),
          (RobotMode.MODE_ADAPTER_ERROR = 8),
          (RobotMode.MODE_CLEANING = 9),
          RobotMode
        );
      })();
      exports.RobotMode = RobotMode;
    },
    '../rmf-models/dist/ros/rmf_fleet_msgs/msg/RobotState.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.RobotState = void 0);
      var RobotMode_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_fleet_msgs/msg/RobotMode.js',
        ),
        Location_1 = __webpack_require__('../rmf-models/dist/ros/rmf_fleet_msgs/msg/Location.js'),
        RobotState = (function () {
          function RobotState(fields) {
            void 0 === fields && (fields = {}),
              (this.name = fields.name || ''),
              (this.model = fields.model || ''),
              (this.task_id = fields.task_id || ''),
              (this.seq = fields.seq || 0),
              (this.mode = fields.mode || new RobotMode_1.RobotMode()),
              (this.battery_percent = fields.battery_percent || 0),
              (this.location = fields.location || new Location_1.Location()),
              (this.path = fields.path || []);
          }
          return (
            (RobotState.validate = function (obj) {
              var e_1, _a;
              if ('string' != typeof obj.name) throw new Error('expected "name" to be "string"');
              if ('string' != typeof obj.model) throw new Error('expected "model" to be "string"');
              if ('string' != typeof obj.task_id)
                throw new Error('expected "task_id" to be "string"');
              if ('number' != typeof obj.seq) throw new Error('expected "seq" to be "number"');
              try {
                RobotMode_1.RobotMode.validate(obj.mode);
              } catch (e) {
                throw new Error('in "mode":\n  ' + e.message);
              }
              if ('number' != typeof obj.battery_percent)
                throw new Error('expected "battery_percent" to be "number"');
              try {
                Location_1.Location.validate(obj.location);
              } catch (e) {
                throw new Error('in "location":\n  ' + e.message);
              }
              if (!Array.isArray(obj.path)) throw new Error('expected "path" to be an array');
              try {
                for (
                  var _b = __values(obj.path.entries()), _c = _b.next();
                  !_c.done;
                  _c = _b.next()
                ) {
                  var _d = __read(_c.value, 2),
                    i = _d[0],
                    v = _d[1];
                  try {
                    Location_1.Location.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "path":\n  ' + e.message);
                  }
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _c && !_c.done && (_a = _b.return) && _a.call(_b);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
            }),
            (RobotState.FullTypeName = 'rmf_fleet_msgs/msg/RobotState'),
            RobotState
          );
        })();
      exports.RobotState = RobotState;
    },
    '../rmf-models/dist/ros/rmf_fleet_msgs/srv/LiftClearance.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.LiftClearance = void 0);
      var LiftClearance_Request_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_fleet_msgs/srv/LiftClearance_Request.js',
        ),
        LiftClearance_Response_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_fleet_msgs/srv/LiftClearance_Response.js',
        ),
        LiftClearance = (function () {
          function LiftClearance() {}
          return (
            (LiftClearance.FullServiceName = 'rmf_fleet_msgs/srv/LiftClearance'),
            (LiftClearance.Request = LiftClearance_Request_1.LiftClearance_Request),
            (LiftClearance.Response = LiftClearance_Response_1.LiftClearance_Response),
            LiftClearance
          );
        })();
      exports.LiftClearance = LiftClearance;
    },
    '../rmf-models/dist/ros/rmf_fleet_msgs/srv/LiftClearance_Request.js': (
      __unused_webpack_module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.LiftClearance_Request = void 0);
      var LiftClearance_Request = (function () {
        function LiftClearance_Request(fields) {
          void 0 === fields && (fields = {}),
            (this.robot_name = fields.robot_name || ''),
            (this.lift_name = fields.lift_name || '');
        }
        return (
          (LiftClearance_Request.validate = function (obj) {
            if ('string' != typeof obj.robot_name)
              throw new Error('expected "robot_name" to be "string"');
            if ('string' != typeof obj.lift_name)
              throw new Error('expected "lift_name" to be "string"');
          }),
          (LiftClearance_Request.FullTypeName = 'rmf_fleet_msgs/srv/LiftClearance_Request'),
          LiftClearance_Request
        );
      })();
      exports.LiftClearance_Request = LiftClearance_Request;
    },
    '../rmf-models/dist/ros/rmf_fleet_msgs/srv/LiftClearance_Response.js': (
      __unused_webpack_module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.LiftClearance_Response = void 0);
      var LiftClearance_Response = (function () {
        function LiftClearance_Response(fields) {
          void 0 === fields && (fields = {}), (this.decision = fields.decision || 0);
        }
        return (
          (LiftClearance_Response.validate = function (obj) {
            if ('number' != typeof obj.decision)
              throw new Error('expected "decision" to be "number"');
          }),
          (LiftClearance_Response.FullTypeName = 'rmf_fleet_msgs/srv/LiftClearance_Response'),
          (LiftClearance_Response.DECISION_CLEAR = 1),
          (LiftClearance_Response.DECISION_CROWDED = 2),
          LiftClearance_Response
        );
      })();
      exports.LiftClearance_Response = LiftClearance_Response;
    },
    '../rmf-models/dist/ros/rmf_ingestor_msgs/msg/IngestorRequest.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.IngestorRequest = void 0);
      var Time_1 = __webpack_require__('../rmf-models/dist/ros/builtin_interfaces/msg/Time.js'),
        IngestorRequestItem_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_ingestor_msgs/msg/IngestorRequestItem.js',
        ),
        IngestorRequest = (function () {
          function IngestorRequest(fields) {
            void 0 === fields && (fields = {}),
              (this.time = fields.time || new Time_1.Time()),
              (this.request_guid = fields.request_guid || ''),
              (this.target_guid = fields.target_guid || ''),
              (this.transporter_type = fields.transporter_type || ''),
              (this.items = fields.items || []);
          }
          return (
            (IngestorRequest.validate = function (obj) {
              var e_1, _a;
              try {
                Time_1.Time.validate(obj.time);
              } catch (e) {
                throw new Error('in "time":\n  ' + e.message);
              }
              if ('string' != typeof obj.request_guid)
                throw new Error('expected "request_guid" to be "string"');
              if ('string' != typeof obj.target_guid)
                throw new Error('expected "target_guid" to be "string"');
              if ('string' != typeof obj.transporter_type)
                throw new Error('expected "transporter_type" to be "string"');
              if (!Array.isArray(obj.items)) throw new Error('expected "items" to be an array');
              try {
                for (
                  var _b = __values(obj.items.entries()), _c = _b.next();
                  !_c.done;
                  _c = _b.next()
                ) {
                  var _d = __read(_c.value, 2),
                    i = _d[0],
                    v = _d[1];
                  try {
                    IngestorRequestItem_1.IngestorRequestItem.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "items":\n  ' + e.message);
                  }
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _c && !_c.done && (_a = _b.return) && _a.call(_b);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
            }),
            (IngestorRequest.FullTypeName = 'rmf_ingestor_msgs/msg/IngestorRequest'),
            IngestorRequest
          );
        })();
      exports.IngestorRequest = IngestorRequest;
    },
    '../rmf-models/dist/ros/rmf_ingestor_msgs/msg/IngestorRequestItem.js': (
      __unused_webpack_module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.IngestorRequestItem = void 0);
      var IngestorRequestItem = (function () {
        function IngestorRequestItem(fields) {
          void 0 === fields && (fields = {}),
            (this.type_guid = fields.type_guid || ''),
            (this.quantity = fields.quantity || 0),
            (this.compartment_name = fields.compartment_name || '');
        }
        return (
          (IngestorRequestItem.validate = function (obj) {
            if ('string' != typeof obj.type_guid)
              throw new Error('expected "type_guid" to be "string"');
            if ('number' != typeof obj.quantity)
              throw new Error('expected "quantity" to be "number"');
            if ('string' != typeof obj.compartment_name)
              throw new Error('expected "compartment_name" to be "string"');
          }),
          (IngestorRequestItem.FullTypeName = 'rmf_ingestor_msgs/msg/IngestorRequestItem'),
          IngestorRequestItem
        );
      })();
      exports.IngestorRequestItem = IngestorRequestItem;
    },
    '../rmf-models/dist/ros/rmf_ingestor_msgs/msg/IngestorResult.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.IngestorResult = void 0);
      var Time_1 = __webpack_require__('../rmf-models/dist/ros/builtin_interfaces/msg/Time.js'),
        IngestorResult = (function () {
          function IngestorResult(fields) {
            void 0 === fields && (fields = {}),
              (this.time = fields.time || new Time_1.Time()),
              (this.request_guid = fields.request_guid || ''),
              (this.source_guid = fields.source_guid || ''),
              (this.status = fields.status || 0);
          }
          return (
            (IngestorResult.validate = function (obj) {
              try {
                Time_1.Time.validate(obj.time);
              } catch (e) {
                throw new Error('in "time":\n  ' + e.message);
              }
              if ('string' != typeof obj.request_guid)
                throw new Error('expected "request_guid" to be "string"');
              if ('string' != typeof obj.source_guid)
                throw new Error('expected "source_guid" to be "string"');
              if ('number' != typeof obj.status)
                throw new Error('expected "status" to be "number"');
            }),
            (IngestorResult.FullTypeName = 'rmf_ingestor_msgs/msg/IngestorResult'),
            (IngestorResult.ACKNOWLEDGED = 0),
            (IngestorResult.SUCCESS = 1),
            (IngestorResult.FAILED = 2),
            IngestorResult
          );
        })();
      exports.IngestorResult = IngestorResult;
    },
    '../rmf-models/dist/ros/rmf_ingestor_msgs/msg/IngestorState.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.IngestorState = void 0);
      var Time_1 = __webpack_require__('../rmf-models/dist/ros/builtin_interfaces/msg/Time.js'),
        IngestorState = (function () {
          function IngestorState(fields) {
            void 0 === fields && (fields = {}),
              (this.time = fields.time || new Time_1.Time()),
              (this.guid = fields.guid || ''),
              (this.mode = fields.mode || 0),
              (this.request_guid_queue = fields.request_guid_queue || []),
              (this.seconds_remaining = fields.seconds_remaining || 0);
          }
          return (
            (IngestorState.validate = function (obj) {
              var e_1, _a;
              try {
                Time_1.Time.validate(obj.time);
              } catch (e) {
                throw new Error('in "time":\n  ' + e.message);
              }
              if ('string' != typeof obj.guid) throw new Error('expected "guid" to be "string"');
              if ('number' != typeof obj.mode) throw new Error('expected "mode" to be "number"');
              if (!Array.isArray(obj.request_guid_queue))
                throw new Error('expected "request_guid_queue" to be an array');
              try {
                for (
                  var _b = __values(obj.request_guid_queue.entries()), _c = _b.next();
                  !_c.done;
                  _c = _b.next()
                ) {
                  var _d = __read(_c.value, 2),
                    i = _d[0];
                  if ('string' != typeof _d[1])
                    throw new Error(
                      'expected index ' + i + ' of "request_guid_queue" to be "string"',
                    );
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _c && !_c.done && (_a = _b.return) && _a.call(_b);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
              if ('number' != typeof obj.seconds_remaining)
                throw new Error('expected "seconds_remaining" to be "number"');
            }),
            (IngestorState.FullTypeName = 'rmf_ingestor_msgs/msg/IngestorState'),
            (IngestorState.IDLE = 0),
            (IngestorState.BUSY = 1),
            (IngestorState.OFFLINE = 2),
            IngestorState
          );
        })();
      exports.IngestorState = IngestorState;
    },
    '../rmf-models/dist/ros/rmf_lift_msgs/msg/LiftRequest.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.LiftRequest = void 0);
      var Time_1 = __webpack_require__('../rmf-models/dist/ros/builtin_interfaces/msg/Time.js'),
        LiftRequest = (function () {
          function LiftRequest(fields) {
            void 0 === fields && (fields = {}),
              (this.lift_name = fields.lift_name || ''),
              (this.request_time = fields.request_time || new Time_1.Time()),
              (this.session_id = fields.session_id || ''),
              (this.request_type = fields.request_type || 0),
              (this.destination_floor = fields.destination_floor || ''),
              (this.door_state = fields.door_state || 0);
          }
          return (
            (LiftRequest.validate = function (obj) {
              if ('string' != typeof obj.lift_name)
                throw new Error('expected "lift_name" to be "string"');
              try {
                Time_1.Time.validate(obj.request_time);
              } catch (e) {
                throw new Error('in "request_time":\n  ' + e.message);
              }
              if ('string' != typeof obj.session_id)
                throw new Error('expected "session_id" to be "string"');
              if ('number' != typeof obj.request_type)
                throw new Error('expected "request_type" to be "number"');
              if ('string' != typeof obj.destination_floor)
                throw new Error('expected "destination_floor" to be "string"');
              if ('number' != typeof obj.door_state)
                throw new Error('expected "door_state" to be "number"');
            }),
            (LiftRequest.FullTypeName = 'rmf_lift_msgs/msg/LiftRequest'),
            (LiftRequest.REQUEST_END_SESSION = 0),
            (LiftRequest.REQUEST_AGV_MODE = 1),
            (LiftRequest.REQUEST_HUMAN_MODE = 2),
            (LiftRequest.DOOR_CLOSED = 0),
            (LiftRequest.DOOR_OPEN = 2),
            LiftRequest
          );
        })();
      exports.LiftRequest = LiftRequest;
    },
    '../rmf-models/dist/ros/rmf_lift_msgs/msg/LiftState.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.LiftState = void 0);
      var Time_1 = __webpack_require__('../rmf-models/dist/ros/builtin_interfaces/msg/Time.js'),
        LiftState = (function () {
          function LiftState(fields) {
            void 0 === fields && (fields = {}),
              (this.lift_time = fields.lift_time || new Time_1.Time()),
              (this.lift_name = fields.lift_name || ''),
              (this.available_floors = fields.available_floors || []),
              (this.current_floor = fields.current_floor || ''),
              (this.destination_floor = fields.destination_floor || ''),
              (this.door_state = fields.door_state || 0),
              (this.motion_state = fields.motion_state || 0),
              (this.available_modes = fields.available_modes || []),
              (this.current_mode = fields.current_mode || 0),
              (this.session_id = fields.session_id || '');
          }
          return (
            (LiftState.validate = function (obj) {
              var e_1, _a, e_2, _b;
              try {
                Time_1.Time.validate(obj.lift_time);
              } catch (e) {
                throw new Error('in "lift_time":\n  ' + e.message);
              }
              if ('string' != typeof obj.lift_name)
                throw new Error('expected "lift_name" to be "string"');
              if (!Array.isArray(obj.available_floors))
                throw new Error('expected "available_floors" to be an array');
              try {
                for (
                  var _c = __values(obj.available_floors.entries()), _d = _c.next();
                  !_d.done;
                  _d = _c.next()
                ) {
                  var _e = __read(_d.value, 2),
                    i = _e[0];
                  if ('string' != typeof _e[1])
                    throw new Error(
                      'expected index ' + i + ' of "available_floors" to be "string"',
                    );
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _d && !_d.done && (_a = _c.return) && _a.call(_c);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
              if ('string' != typeof obj.current_floor)
                throw new Error('expected "current_floor" to be "string"');
              if ('string' != typeof obj.destination_floor)
                throw new Error('expected "destination_floor" to be "string"');
              if ('number' != typeof obj.door_state)
                throw new Error('expected "door_state" to be "number"');
              if ('number' != typeof obj.motion_state)
                throw new Error('expected "motion_state" to be "number"');
              if (
                !(obj.available_modes instanceof Uint8Array || Array.isArray(obj.available_modes))
              )
                throw new Error('expected "available_modes" to be "Uint8Array" or an array');
              if (Array.isArray(obj.available_modes))
                try {
                  for (
                    var _f = __values(obj.available_modes.entries()), _g = _f.next();
                    !_g.done;
                    _g = _f.next()
                  ) {
                    var _h = __read(_g.value, 2);
                    i = _h[0];
                    if ('number' != typeof _h[1])
                      throw new Error(
                        'expected index ' + i + ' of "available_modes" to be "number"',
                      );
                  }
                } catch (e_2_1) {
                  e_2 = { error: e_2_1 };
                } finally {
                  try {
                    _g && !_g.done && (_b = _f.return) && _b.call(_f);
                  } finally {
                    if (e_2) throw e_2.error;
                  }
                }
              if ('number' != typeof obj.current_mode)
                throw new Error('expected "current_mode" to be "number"');
              if ('string' != typeof obj.session_id)
                throw new Error('expected "session_id" to be "string"');
            }),
            (LiftState.FullTypeName = 'rmf_lift_msgs/msg/LiftState'),
            (LiftState.DOOR_CLOSED = 0),
            (LiftState.DOOR_MOVING = 1),
            (LiftState.DOOR_OPEN = 2),
            (LiftState.MOTION_STOPPED = 0),
            (LiftState.MOTION_UP = 1),
            (LiftState.MOTION_DOWN = 2),
            (LiftState.MOTION_UNKNOWN = 3),
            (LiftState.MODE_UNKNOWN = 0),
            (LiftState.MODE_HUMAN = 1),
            (LiftState.MODE_AGV = 2),
            (LiftState.MODE_FIRE = 3),
            (LiftState.MODE_OFFLINE = 4),
            (LiftState.MODE_EMERGENCY = 5),
            LiftState
          );
        })();
      exports.LiftState = LiftState;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/msg/Behavior.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.Behavior = void 0);
      var BehaviorParameter_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_task_msgs/msg/BehaviorParameter.js',
        ),
        Behavior = (function () {
          function Behavior(fields) {
            void 0 === fields && (fields = {}),
              (this.name = fields.name || ''),
              (this.parameters = fields.parameters || []);
          }
          return (
            (Behavior.validate = function (obj) {
              var e_1, _a;
              if ('string' != typeof obj.name) throw new Error('expected "name" to be "string"');
              if (!Array.isArray(obj.parameters))
                throw new Error('expected "parameters" to be an array');
              try {
                for (
                  var _b = __values(obj.parameters.entries()), _c = _b.next();
                  !_c.done;
                  _c = _b.next()
                ) {
                  var _d = __read(_c.value, 2),
                    i = _d[0],
                    v = _d[1];
                  try {
                    BehaviorParameter_1.BehaviorParameter.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "parameters":\n  ' + e.message);
                  }
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _c && !_c.done && (_a = _b.return) && _a.call(_b);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
            }),
            (Behavior.FullTypeName = 'rmf_task_msgs/msg/Behavior'),
            Behavior
          );
        })();
      exports.Behavior = Behavior;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/msg/BehaviorParameter.js': (
      __unused_webpack_module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.BehaviorParameter = void 0);
      var BehaviorParameter = (function () {
        function BehaviorParameter(fields) {
          void 0 === fields && (fields = {}),
            (this.name = fields.name || ''),
            (this.value = fields.value || '');
        }
        return (
          (BehaviorParameter.validate = function (obj) {
            if ('string' != typeof obj.name) throw new Error('expected "name" to be "string"');
            if ('string' != typeof obj.value) throw new Error('expected "value" to be "string"');
          }),
          (BehaviorParameter.FullTypeName = 'rmf_task_msgs/msg/BehaviorParameter'),
          BehaviorParameter
        );
      })();
      exports.BehaviorParameter = BehaviorParameter;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/msg/BidNotice.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.BidNotice = void 0);
      var TaskProfile_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_task_msgs/msg/TaskProfile.js',
        ),
        Duration_1 = __webpack_require__(
          '../rmf-models/dist/ros/builtin_interfaces/msg/Duration.js',
        ),
        BidNotice = (function () {
          function BidNotice(fields) {
            void 0 === fields && (fields = {}),
              (this.task_profile = fields.task_profile || new TaskProfile_1.TaskProfile()),
              (this.time_window = fields.time_window || new Duration_1.Duration());
          }
          return (
            (BidNotice.validate = function (obj) {
              try {
                TaskProfile_1.TaskProfile.validate(obj.task_profile);
              } catch (e) {
                throw new Error('in "task_profile":\n  ' + e.message);
              }
              try {
                Duration_1.Duration.validate(obj.time_window);
              } catch (e) {
                throw new Error('in "time_window":\n  ' + e.message);
              }
            }),
            (BidNotice.FullTypeName = 'rmf_task_msgs/msg/BidNotice'),
            BidNotice
          );
        })();
      exports.BidNotice = BidNotice;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/msg/BidProposal.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.BidProposal = void 0);
      var TaskProfile_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_task_msgs/msg/TaskProfile.js',
        ),
        Time_1 = __webpack_require__('../rmf-models/dist/ros/builtin_interfaces/msg/Time.js'),
        BidProposal = (function () {
          function BidProposal(fields) {
            void 0 === fields && (fields = {}),
              (this.fleet_name = fields.fleet_name || ''),
              (this.task_profile = fields.task_profile || new TaskProfile_1.TaskProfile()),
              (this.prev_cost = fields.prev_cost || 0),
              (this.new_cost = fields.new_cost || 0),
              (this.finish_time = fields.finish_time || new Time_1.Time()),
              (this.robot_name = fields.robot_name || '');
          }
          return (
            (BidProposal.validate = function (obj) {
              if ('string' != typeof obj.fleet_name)
                throw new Error('expected "fleet_name" to be "string"');
              try {
                TaskProfile_1.TaskProfile.validate(obj.task_profile);
              } catch (e) {
                throw new Error('in "task_profile":\n  ' + e.message);
              }
              if ('number' != typeof obj.prev_cost)
                throw new Error('expected "prev_cost" to be "number"');
              if ('number' != typeof obj.new_cost)
                throw new Error('expected "new_cost" to be "number"');
              try {
                Time_1.Time.validate(obj.finish_time);
              } catch (e) {
                throw new Error('in "finish_time":\n  ' + e.message);
              }
              if ('string' != typeof obj.robot_name)
                throw new Error('expected "robot_name" to be "string"');
            }),
            (BidProposal.FullTypeName = 'rmf_task_msgs/msg/BidProposal'),
            BidProposal
          );
        })();
      exports.BidProposal = BidProposal;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/msg/Clean.js': (__unused_webpack_module, exports) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.Clean = void 0);
      var Clean = (function () {
        function Clean(fields) {
          void 0 === fields && (fields = {}), (this.start_waypoint = fields.start_waypoint || '');
        }
        return (
          (Clean.validate = function (obj) {
            if ('string' != typeof obj.start_waypoint)
              throw new Error('expected "start_waypoint" to be "string"');
          }),
          (Clean.FullTypeName = 'rmf_task_msgs/msg/Clean'),
          Clean
        );
      })();
      exports.Clean = Clean;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/msg/Delivery.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.Delivery = void 0);
      var DispenserRequestItem_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_dispenser_msgs/msg/DispenserRequestItem.js',
        ),
        Behavior_1 = __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/Behavior.js'),
        Delivery = (function () {
          function Delivery(fields) {
            void 0 === fields && (fields = {}),
              (this.task_id = fields.task_id || ''),
              (this.items = fields.items || []),
              (this.pickup_place_name = fields.pickup_place_name || ''),
              (this.pickup_dispenser = fields.pickup_dispenser || ''),
              (this.pickup_behavior = fields.pickup_behavior || new Behavior_1.Behavior()),
              (this.dropoff_place_name = fields.dropoff_place_name || ''),
              (this.dropoff_ingestor = fields.dropoff_ingestor || ''),
              (this.dropoff_behavior = fields.dropoff_behavior || new Behavior_1.Behavior());
          }
          return (
            (Delivery.validate = function (obj) {
              var e_1, _a;
              if ('string' != typeof obj.task_id)
                throw new Error('expected "task_id" to be "string"');
              if (!Array.isArray(obj.items)) throw new Error('expected "items" to be an array');
              try {
                for (
                  var _b = __values(obj.items.entries()), _c = _b.next();
                  !_c.done;
                  _c = _b.next()
                ) {
                  var _d = __read(_c.value, 2),
                    i = _d[0],
                    v = _d[1];
                  try {
                    DispenserRequestItem_1.DispenserRequestItem.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "items":\n  ' + e.message);
                  }
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _c && !_c.done && (_a = _b.return) && _a.call(_b);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
              if ('string' != typeof obj.pickup_place_name)
                throw new Error('expected "pickup_place_name" to be "string"');
              if ('string' != typeof obj.pickup_dispenser)
                throw new Error('expected "pickup_dispenser" to be "string"');
              try {
                Behavior_1.Behavior.validate(obj.pickup_behavior);
              } catch (e) {
                throw new Error('in "pickup_behavior":\n  ' + e.message);
              }
              if ('string' != typeof obj.dropoff_place_name)
                throw new Error('expected "dropoff_place_name" to be "string"');
              if ('string' != typeof obj.dropoff_ingestor)
                throw new Error('expected "dropoff_ingestor" to be "string"');
              try {
                Behavior_1.Behavior.validate(obj.dropoff_behavior);
              } catch (e) {
                throw new Error('in "dropoff_behavior":\n  ' + e.message);
              }
            }),
            (Delivery.FullTypeName = 'rmf_task_msgs/msg/Delivery'),
            Delivery
          );
        })();
      exports.Delivery = Delivery;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/msg/DispatchAck.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.DispatchAck = void 0);
      var DispatchRequest_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_task_msgs/msg/DispatchRequest.js',
        ),
        DispatchAck = (function () {
          function DispatchAck(fields) {
            void 0 === fields && (fields = {}),
              (this.dispatch_request =
                fields.dispatch_request || new DispatchRequest_1.DispatchRequest()),
              (this.success = fields.success || !1);
          }
          return (
            (DispatchAck.validate = function (obj) {
              try {
                DispatchRequest_1.DispatchRequest.validate(obj.dispatch_request);
              } catch (e) {
                throw new Error('in "dispatch_request":\n  ' + e.message);
              }
              if ('boolean' != typeof obj.success)
                throw new Error('expected "success" to be "boolean"');
            }),
            (DispatchAck.FullTypeName = 'rmf_task_msgs/msg/DispatchAck'),
            DispatchAck
          );
        })();
      exports.DispatchAck = DispatchAck;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/msg/DispatchRequest.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.DispatchRequest = void 0);
      var TaskProfile_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_task_msgs/msg/TaskProfile.js',
        ),
        DispatchRequest = (function () {
          function DispatchRequest(fields) {
            void 0 === fields && (fields = {}),
              (this.fleet_name = fields.fleet_name || ''),
              (this.task_profile = fields.task_profile || new TaskProfile_1.TaskProfile()),
              (this.method = fields.method || 0);
          }
          return (
            (DispatchRequest.validate = function (obj) {
              if ('string' != typeof obj.fleet_name)
                throw new Error('expected "fleet_name" to be "string"');
              try {
                TaskProfile_1.TaskProfile.validate(obj.task_profile);
              } catch (e) {
                throw new Error('in "task_profile":\n  ' + e.message);
              }
              if ('number' != typeof obj.method)
                throw new Error('expected "method" to be "number"');
            }),
            (DispatchRequest.FullTypeName = 'rmf_task_msgs/msg/DispatchRequest'),
            (DispatchRequest.ADD = 1),
            (DispatchRequest.CANCEL = 2),
            DispatchRequest
          );
        })();
      exports.DispatchRequest = DispatchRequest;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/msg/Loop.js': (__unused_webpack_module, exports) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.Loop = void 0);
      var Loop = (function () {
        function Loop(fields) {
          void 0 === fields && (fields = {}),
            (this.task_id = fields.task_id || ''),
            (this.robot_type = fields.robot_type || ''),
            (this.num_loops = fields.num_loops || 0),
            (this.start_name = fields.start_name || ''),
            (this.finish_name = fields.finish_name || '');
        }
        return (
          (Loop.validate = function (obj) {
            if ('string' != typeof obj.task_id)
              throw new Error('expected "task_id" to be "string"');
            if ('string' != typeof obj.robot_type)
              throw new Error('expected "robot_type" to be "string"');
            if ('number' != typeof obj.num_loops)
              throw new Error('expected "num_loops" to be "number"');
            if ('string' != typeof obj.start_name)
              throw new Error('expected "start_name" to be "string"');
            if ('string' != typeof obj.finish_name)
              throw new Error('expected "finish_name" to be "string"');
          }),
          (Loop.FullTypeName = 'rmf_task_msgs/msg/Loop'),
          Loop
        );
      })();
      exports.Loop = Loop;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/msg/Priority.js': (__unused_webpack_module, exports) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.Priority = void 0);
      var Priority = (function () {
        function Priority(fields) {
          void 0 === fields && (fields = {}), (this.value = fields.value || 0);
        }
        return (
          (Priority.validate = function (obj) {
            if ('number' != typeof obj.value) throw new Error('expected "value" to be "number"');
          }),
          (Priority.FullTypeName = 'rmf_task_msgs/msg/Priority'),
          Priority
        );
      })();
      exports.Priority = Priority;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/msg/Station.js': (__unused_webpack_module, exports) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.Station = void 0);
      var Station = (function () {
        function Station(fields) {
          void 0 === fields && (fields = {}),
            (this.task_id = fields.task_id || ''),
            (this.robot_type = fields.robot_type || ''),
            (this.place_name = fields.place_name || '');
        }
        return (
          (Station.validate = function (obj) {
            if ('string' != typeof obj.task_id)
              throw new Error('expected "task_id" to be "string"');
            if ('string' != typeof obj.robot_type)
              throw new Error('expected "robot_type" to be "string"');
            if ('string' != typeof obj.place_name)
              throw new Error('expected "place_name" to be "string"');
          }),
          (Station.FullTypeName = 'rmf_task_msgs/msg/Station'),
          Station
        );
      })();
      exports.Station = Station;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/msg/TaskDescription.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.TaskDescription = void 0);
      var Time_1 = __webpack_require__('../rmf-models/dist/ros/builtin_interfaces/msg/Time.js'),
        Priority_1 = __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/Priority.js'),
        TaskType_1 = __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/TaskType.js'),
        Station_1 = __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/Station.js'),
        Loop_1 = __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/Loop.js'),
        Delivery_1 = __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/Delivery.js'),
        Clean_1 = __webpack_require__('../rmf-models/dist/ros/rmf_task_msgs/msg/Clean.js'),
        TaskDescription = (function () {
          function TaskDescription(fields) {
            void 0 === fields && (fields = {}),
              (this.start_time = fields.start_time || new Time_1.Time()),
              (this.priority = fields.priority || new Priority_1.Priority()),
              (this.task_type = fields.task_type || new TaskType_1.TaskType()),
              (this.station = fields.station || new Station_1.Station()),
              (this.loop = fields.loop || new Loop_1.Loop()),
              (this.delivery = fields.delivery || new Delivery_1.Delivery()),
              (this.clean = fields.clean || new Clean_1.Clean());
          }
          return (
            (TaskDescription.validate = function (obj) {
              try {
                Time_1.Time.validate(obj.start_time);
              } catch (e) {
                throw new Error('in "start_time":\n  ' + e.message);
              }
              try {
                Priority_1.Priority.validate(obj.priority);
              } catch (e) {
                throw new Error('in "priority":\n  ' + e.message);
              }
              try {
                TaskType_1.TaskType.validate(obj.task_type);
              } catch (e) {
                throw new Error('in "task_type":\n  ' + e.message);
              }
              try {
                Station_1.Station.validate(obj.station);
              } catch (e) {
                throw new Error('in "station":\n  ' + e.message);
              }
              try {
                Loop_1.Loop.validate(obj.loop);
              } catch (e) {
                throw new Error('in "loop":\n  ' + e.message);
              }
              try {
                Delivery_1.Delivery.validate(obj.delivery);
              } catch (e) {
                throw new Error('in "delivery":\n  ' + e.message);
              }
              try {
                Clean_1.Clean.validate(obj.clean);
              } catch (e) {
                throw new Error('in "clean":\n  ' + e.message);
              }
            }),
            (TaskDescription.FullTypeName = 'rmf_task_msgs/msg/TaskDescription'),
            TaskDescription
          );
        })();
      exports.TaskDescription = TaskDescription;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/msg/TaskProfile.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.TaskProfile = void 0);
      var Time_1 = __webpack_require__('../rmf-models/dist/ros/builtin_interfaces/msg/Time.js'),
        TaskDescription_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_task_msgs/msg/TaskDescription.js',
        ),
        TaskProfile = (function () {
          function TaskProfile(fields) {
            void 0 === fields && (fields = {}),
              (this.task_id = fields.task_id || ''),
              (this.submission_time = fields.submission_time || new Time_1.Time()),
              (this.description = fields.description || new TaskDescription_1.TaskDescription());
          }
          return (
            (TaskProfile.validate = function (obj) {
              if ('string' != typeof obj.task_id)
                throw new Error('expected "task_id" to be "string"');
              try {
                Time_1.Time.validate(obj.submission_time);
              } catch (e) {
                throw new Error('in "submission_time":\n  ' + e.message);
              }
              try {
                TaskDescription_1.TaskDescription.validate(obj.description);
              } catch (e) {
                throw new Error('in "description":\n  ' + e.message);
              }
            }),
            (TaskProfile.FullTypeName = 'rmf_task_msgs/msg/TaskProfile'),
            TaskProfile
          );
        })();
      exports.TaskProfile = TaskProfile;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/msg/TaskSummary.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.TaskSummary = void 0);
      var TaskProfile_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_task_msgs/msg/TaskProfile.js',
        ),
        Time_1 = __webpack_require__('../rmf-models/dist/ros/builtin_interfaces/msg/Time.js'),
        TaskSummary = (function () {
          function TaskSummary(fields) {
            void 0 === fields && (fields = {}),
              (this.fleet_name = fields.fleet_name || ''),
              (this.task_id = fields.task_id || ''),
              (this.task_profile = fields.task_profile || new TaskProfile_1.TaskProfile()),
              (this.state = fields.state || 0),
              (this.status = fields.status || ''),
              (this.submission_time = fields.submission_time || new Time_1.Time()),
              (this.start_time = fields.start_time || new Time_1.Time()),
              (this.end_time = fields.end_time || new Time_1.Time()),
              (this.robot_name = fields.robot_name || '');
          }
          return (
            (TaskSummary.validate = function (obj) {
              if ('string' != typeof obj.fleet_name)
                throw new Error('expected "fleet_name" to be "string"');
              if ('string' != typeof obj.task_id)
                throw new Error('expected "task_id" to be "string"');
              try {
                TaskProfile_1.TaskProfile.validate(obj.task_profile);
              } catch (e) {
                throw new Error('in "task_profile":\n  ' + e.message);
              }
              if ('number' != typeof obj.state) throw new Error('expected "state" to be "number"');
              if ('string' != typeof obj.status)
                throw new Error('expected "status" to be "string"');
              try {
                Time_1.Time.validate(obj.submission_time);
              } catch (e) {
                throw new Error('in "submission_time":\n  ' + e.message);
              }
              try {
                Time_1.Time.validate(obj.start_time);
              } catch (e) {
                throw new Error('in "start_time":\n  ' + e.message);
              }
              try {
                Time_1.Time.validate(obj.end_time);
              } catch (e) {
                throw new Error('in "end_time":\n  ' + e.message);
              }
              if ('string' != typeof obj.robot_name)
                throw new Error('expected "robot_name" to be "string"');
            }),
            (TaskSummary.FullTypeName = 'rmf_task_msgs/msg/TaskSummary'),
            (TaskSummary.STATE_QUEUED = 0),
            (TaskSummary.STATE_ACTIVE = 1),
            (TaskSummary.STATE_COMPLETED = 2),
            (TaskSummary.STATE_FAILED = 3),
            (TaskSummary.STATE_CANCELED = 4),
            (TaskSummary.STATE_PENDING = 5),
            TaskSummary
          );
        })();
      exports.TaskSummary = TaskSummary;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/msg/TaskType.js': (__unused_webpack_module, exports) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.TaskType = void 0);
      var TaskType = (function () {
        function TaskType(fields) {
          void 0 === fields && (fields = {}), (this.type = fields.type || 0);
        }
        return (
          (TaskType.validate = function (obj) {
            if ('number' != typeof obj.type) throw new Error('expected "type" to be "number"');
          }),
          (TaskType.FullTypeName = 'rmf_task_msgs/msg/TaskType'),
          (TaskType.TYPE_STATION = 0),
          (TaskType.TYPE_LOOP = 1),
          (TaskType.TYPE_DELIVERY = 2),
          (TaskType.TYPE_CHARGE_BATTERY = 3),
          (TaskType.TYPE_CLEAN = 4),
          (TaskType.TYPE_PATROL = 5),
          TaskType
        );
      })();
      exports.TaskType = TaskType;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/msg/Tasks.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.Tasks = void 0);
      var TaskSummary_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_task_msgs/msg/TaskSummary.js',
        ),
        Tasks = (function () {
          function Tasks(fields) {
            void 0 === fields && (fields = {}), (this.tasks = fields.tasks || []);
          }
          return (
            (Tasks.validate = function (obj) {
              var e_1, _a;
              if (!Array.isArray(obj.tasks)) throw new Error('expected "tasks" to be an array');
              try {
                for (
                  var _b = __values(obj.tasks.entries()), _c = _b.next();
                  !_c.done;
                  _c = _b.next()
                ) {
                  var _d = __read(_c.value, 2),
                    i = _d[0],
                    v = _d[1];
                  try {
                    TaskSummary_1.TaskSummary.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "tasks":\n  ' + e.message);
                  }
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _c && !_c.done && (_a = _b.return) && _a.call(_b);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
            }),
            (Tasks.FullTypeName = 'rmf_task_msgs/msg/Tasks'),
            Tasks
          );
        })();
      exports.Tasks = Tasks;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/msg/Tow.js': (__unused_webpack_module, exports) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.Tow = void 0);
      var Tow = (function () {
        function Tow(fields) {
          void 0 === fields && (fields = {}),
            (this.task_id = fields.task_id || ''),
            (this.object_type = fields.object_type || ''),
            (this.is_object_id_known = fields.is_object_id_known || !1),
            (this.object_id = fields.object_id || ''),
            (this.pickup_place_name = fields.pickup_place_name || ''),
            (this.is_dropoff_place_known = fields.is_dropoff_place_known || !1),
            (this.dropoff_place_name = fields.dropoff_place_name || '');
        }
        return (
          (Tow.validate = function (obj) {
            if ('string' != typeof obj.task_id)
              throw new Error('expected "task_id" to be "string"');
            if ('string' != typeof obj.object_type)
              throw new Error('expected "object_type" to be "string"');
            if ('boolean' != typeof obj.is_object_id_known)
              throw new Error('expected "is_object_id_known" to be "boolean"');
            if ('string' != typeof obj.object_id)
              throw new Error('expected "object_id" to be "string"');
            if ('string' != typeof obj.pickup_place_name)
              throw new Error('expected "pickup_place_name" to be "string"');
            if ('boolean' != typeof obj.is_dropoff_place_known)
              throw new Error('expected "is_dropoff_place_known" to be "boolean"');
            if ('string' != typeof obj.dropoff_place_name)
              throw new Error('expected "dropoff_place_name" to be "string"');
          }),
          (Tow.FullTypeName = 'rmf_task_msgs/msg/Tow'),
          Tow
        );
      })();
      exports.Tow = Tow;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/srv/CancelTask.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.CancelTask = void 0);
      var CancelTask_Request_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_task_msgs/srv/CancelTask_Request.js',
        ),
        CancelTask_Response_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_task_msgs/srv/CancelTask_Response.js',
        ),
        CancelTask = (function () {
          function CancelTask() {}
          return (
            (CancelTask.FullServiceName = 'rmf_task_msgs/srv/CancelTask'),
            (CancelTask.Request = CancelTask_Request_1.CancelTask_Request),
            (CancelTask.Response = CancelTask_Response_1.CancelTask_Response),
            CancelTask
          );
        })();
      exports.CancelTask = CancelTask;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/srv/CancelTask_Request.js': (
      __unused_webpack_module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.CancelTask_Request = void 0);
      var CancelTask_Request = (function () {
        function CancelTask_Request(fields) {
          void 0 === fields && (fields = {}),
            (this.requester = fields.requester || ''),
            (this.task_id = fields.task_id || '');
        }
        return (
          (CancelTask_Request.validate = function (obj) {
            if ('string' != typeof obj.requester)
              throw new Error('expected "requester" to be "string"');
            if ('string' != typeof obj.task_id)
              throw new Error('expected "task_id" to be "string"');
          }),
          (CancelTask_Request.FullTypeName = 'rmf_task_msgs/srv/CancelTask_Request'),
          CancelTask_Request
        );
      })();
      exports.CancelTask_Request = CancelTask_Request;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/srv/CancelTask_Response.js': (
      __unused_webpack_module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.CancelTask_Response = void 0);
      var CancelTask_Response = (function () {
        function CancelTask_Response(fields) {
          void 0 === fields && (fields = {}),
            (this.success = fields.success || !1),
            (this.message = fields.message || '');
        }
        return (
          (CancelTask_Response.validate = function (obj) {
            if ('boolean' != typeof obj.success)
              throw new Error('expected "success" to be "boolean"');
            if ('string' != typeof obj.message)
              throw new Error('expected "message" to be "string"');
          }),
          (CancelTask_Response.FullTypeName = 'rmf_task_msgs/srv/CancelTask_Response'),
          CancelTask_Response
        );
      })();
      exports.CancelTask_Response = CancelTask_Response;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/srv/GetTaskList.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.GetTaskList = void 0);
      var GetTaskList_Request_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_task_msgs/srv/GetTaskList_Request.js',
        ),
        GetTaskList_Response_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_task_msgs/srv/GetTaskList_Response.js',
        ),
        GetTaskList = (function () {
          function GetTaskList() {}
          return (
            (GetTaskList.FullServiceName = 'rmf_task_msgs/srv/GetTaskList'),
            (GetTaskList.Request = GetTaskList_Request_1.GetTaskList_Request),
            (GetTaskList.Response = GetTaskList_Response_1.GetTaskList_Response),
            GetTaskList
          );
        })();
      exports.GetTaskList = GetTaskList;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/srv/GetTaskList_Request.js': function (
      __unused_webpack_module,
      exports,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.GetTaskList_Request = void 0);
      var GetTaskList_Request = (function () {
        function GetTaskList_Request(fields) {
          void 0 === fields && (fields = {}),
            (this.requester = fields.requester || ''),
            (this.task_id = fields.task_id || []);
        }
        return (
          (GetTaskList_Request.validate = function (obj) {
            var e_1, _a;
            if ('string' != typeof obj.requester)
              throw new Error('expected "requester" to be "string"');
            if (!Array.isArray(obj.task_id)) throw new Error('expected "task_id" to be an array');
            try {
              for (
                var _b = __values(obj.task_id.entries()), _c = _b.next();
                !_c.done;
                _c = _b.next()
              ) {
                var _d = __read(_c.value, 2),
                  i = _d[0];
                if ('string' != typeof _d[1])
                  throw new Error('expected index ' + i + ' of "task_id" to be "string"');
              }
            } catch (e_1_1) {
              e_1 = { error: e_1_1 };
            } finally {
              try {
                _c && !_c.done && (_a = _b.return) && _a.call(_b);
              } finally {
                if (e_1) throw e_1.error;
              }
            }
          }),
          (GetTaskList_Request.FullTypeName = 'rmf_task_msgs/srv/GetTaskList_Request'),
          GetTaskList_Request
        );
      })();
      exports.GetTaskList_Request = GetTaskList_Request;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/srv/GetTaskList_Response.js': function (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) {
      'use strict';
      var __values =
          (this && this.__values) ||
          function (o) {
            var s = 'function' == typeof Symbol && Symbol.iterator,
              m = s && o[s],
              i = 0;
            if (m) return m.call(o);
            if (o && 'number' == typeof o.length)
              return {
                next: function () {
                  return o && i >= o.length && (o = void 0), { value: o && o[i++], done: !o };
                },
              };
            throw new TypeError(s ? 'Object is not iterable.' : 'Symbol.iterator is not defined.');
          },
        __read =
          (this && this.__read) ||
          function (o, n) {
            var m = 'function' == typeof Symbol && o[Symbol.iterator];
            if (!m) return o;
            var r,
              e,
              i = m.call(o),
              ar = [];
            try {
              for (; (void 0 === n || n-- > 0) && !(r = i.next()).done; ) ar.push(r.value);
            } catch (error) {
              e = { error };
            } finally {
              try {
                r && !r.done && (m = i.return) && m.call(i);
              } finally {
                if (e) throw e.error;
              }
            }
            return ar;
          };
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.GetTaskList_Response = void 0);
      var TaskSummary_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_task_msgs/msg/TaskSummary.js',
        ),
        GetTaskList_Response = (function () {
          function GetTaskList_Response(fields) {
            void 0 === fields && (fields = {}),
              (this.success = fields.success || !1),
              (this.active_tasks = fields.active_tasks || []),
              (this.terminated_tasks = fields.terminated_tasks || []);
          }
          return (
            (GetTaskList_Response.validate = function (obj) {
              var e_1, _a, e_2, _b;
              if ('boolean' != typeof obj.success)
                throw new Error('expected "success" to be "boolean"');
              if (!Array.isArray(obj.active_tasks))
                throw new Error('expected "active_tasks" to be an array');
              try {
                for (
                  var _c = __values(obj.active_tasks.entries()), _d = _c.next();
                  !_d.done;
                  _d = _c.next()
                ) {
                  var _e = __read(_d.value, 2),
                    i = _e[0],
                    v = _e[1];
                  try {
                    TaskSummary_1.TaskSummary.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "active_tasks":\n  ' + e.message);
                  }
                }
              } catch (e_1_1) {
                e_1 = { error: e_1_1 };
              } finally {
                try {
                  _d && !_d.done && (_a = _c.return) && _a.call(_c);
                } finally {
                  if (e_1) throw e_1.error;
                }
              }
              if (!Array.isArray(obj.terminated_tasks))
                throw new Error('expected "terminated_tasks" to be an array');
              try {
                for (
                  var _f = __values(obj.terminated_tasks.entries()), _g = _f.next();
                  !_g.done;
                  _g = _f.next()
                ) {
                  var _h = __read(_g.value, 2);
                  (i = _h[0]), (v = _h[1]);
                  try {
                    TaskSummary_1.TaskSummary.validate(v);
                  } catch (e) {
                    throw new Error('in index ' + i + ' of "terminated_tasks":\n  ' + e.message);
                  }
                }
              } catch (e_2_1) {
                e_2 = { error: e_2_1 };
              } finally {
                try {
                  _g && !_g.done && (_b = _f.return) && _b.call(_f);
                } finally {
                  if (e_2) throw e_2.error;
                }
              }
            }),
            (GetTaskList_Response.FullTypeName = 'rmf_task_msgs/srv/GetTaskList_Response'),
            GetTaskList_Response
          );
        })();
      exports.GetTaskList_Response = GetTaskList_Response;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/srv/ReviveTask.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.ReviveTask = void 0);
      var ReviveTask_Request_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_task_msgs/srv/ReviveTask_Request.js',
        ),
        ReviveTask_Response_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_task_msgs/srv/ReviveTask_Response.js',
        ),
        ReviveTask = (function () {
          function ReviveTask() {}
          return (
            (ReviveTask.FullServiceName = 'rmf_task_msgs/srv/ReviveTask'),
            (ReviveTask.Request = ReviveTask_Request_1.ReviveTask_Request),
            (ReviveTask.Response = ReviveTask_Response_1.ReviveTask_Response),
            ReviveTask
          );
        })();
      exports.ReviveTask = ReviveTask;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/srv/ReviveTask_Request.js': (
      __unused_webpack_module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.ReviveTask_Request = void 0);
      var ReviveTask_Request = (function () {
        function ReviveTask_Request(fields) {
          void 0 === fields && (fields = {}),
            (this.requester = fields.requester || ''),
            (this.task_id = fields.task_id || '');
        }
        return (
          (ReviveTask_Request.validate = function (obj) {
            if ('string' != typeof obj.requester)
              throw new Error('expected "requester" to be "string"');
            if ('string' != typeof obj.task_id)
              throw new Error('expected "task_id" to be "string"');
          }),
          (ReviveTask_Request.FullTypeName = 'rmf_task_msgs/srv/ReviveTask_Request'),
          ReviveTask_Request
        );
      })();
      exports.ReviveTask_Request = ReviveTask_Request;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/srv/ReviveTask_Response.js': (
      __unused_webpack_module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.ReviveTask_Response = void 0);
      var ReviveTask_Response = (function () {
        function ReviveTask_Response(fields) {
          void 0 === fields && (fields = {}), (this.success = fields.success || !1);
        }
        return (
          (ReviveTask_Response.validate = function (obj) {
            if ('boolean' != typeof obj.success)
              throw new Error('expected "success" to be "boolean"');
          }),
          (ReviveTask_Response.FullTypeName = 'rmf_task_msgs/srv/ReviveTask_Response'),
          ReviveTask_Response
        );
      })();
      exports.ReviveTask_Response = ReviveTask_Response;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/srv/SubmitTask.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.SubmitTask = void 0);
      var SubmitTask_Request_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_task_msgs/srv/SubmitTask_Request.js',
        ),
        SubmitTask_Response_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_task_msgs/srv/SubmitTask_Response.js',
        ),
        SubmitTask = (function () {
          function SubmitTask() {}
          return (
            (SubmitTask.FullServiceName = 'rmf_task_msgs/srv/SubmitTask'),
            (SubmitTask.Request = SubmitTask_Request_1.SubmitTask_Request),
            (SubmitTask.Response = SubmitTask_Response_1.SubmitTask_Response),
            SubmitTask
          );
        })();
      exports.SubmitTask = SubmitTask;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/srv/SubmitTask_Request.js': (
      __unused_webpack_module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.SubmitTask_Request = void 0);
      var TaskDescription_1 = __webpack_require__(
          '../rmf-models/dist/ros/rmf_task_msgs/msg/TaskDescription.js',
        ),
        SubmitTask_Request = (function () {
          function SubmitTask_Request(fields) {
            void 0 === fields && (fields = {}),
              (this.requester = fields.requester || ''),
              (this.description = fields.description || new TaskDescription_1.TaskDescription());
          }
          return (
            (SubmitTask_Request.validate = function (obj) {
              if ('string' != typeof obj.requester)
                throw new Error('expected "requester" to be "string"');
              try {
                TaskDescription_1.TaskDescription.validate(obj.description);
              } catch (e) {
                throw new Error('in "description":\n  ' + e.message);
              }
            }),
            (SubmitTask_Request.FullTypeName = 'rmf_task_msgs/srv/SubmitTask_Request'),
            SubmitTask_Request
          );
        })();
      exports.SubmitTask_Request = SubmitTask_Request;
    },
    '../rmf-models/dist/ros/rmf_task_msgs/srv/SubmitTask_Response.js': (
      __unused_webpack_module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.SubmitTask_Response = void 0);
      var SubmitTask_Response = (function () {
        function SubmitTask_Response(fields) {
          void 0 === fields && (fields = {}),
            (this.success = fields.success || !1),
            (this.task_id = fields.task_id || ''),
            (this.message = fields.message || '');
        }
        return (
          (SubmitTask_Response.validate = function (obj) {
            if ('boolean' != typeof obj.success)
              throw new Error('expected "success" to be "boolean"');
            if ('string' != typeof obj.task_id)
              throw new Error('expected "task_id" to be "string"');
            if ('string' != typeof obj.message)
              throw new Error('expected "message" to be "string"');
          }),
          (SubmitTask_Response.FullTypeName = 'rmf_task_msgs/srv/SubmitTask_Response'),
          SubmitTask_Response
        );
      })();
      exports.SubmitTask_Response = SubmitTask_Response;
    },
    '../rmf-models/dist/version.js': (__unused_webpack_module, exports) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.version = void 0),
        (exports.version = {
          rmf_internal_msgs: '0c237e1758872917661879975d7dc0acf5fa518c',
          rmf_building_map_msgs: '1.2.0',
          rmf_server: 'bee64843943df02ddbea44659343a5f9ba2c9e64',
        });
    },
    './src/assets eager recursive ^\\.\\/.*$': (
      module,
      __unused_webpack_exports,
      __webpack_require__,
    ) => {
      var map = { './defaultLogo.png': './src/assets/defaultLogo.png' };
      function webpackAsyncContext(req) {
        return webpackAsyncContextResolve(req).then((id) => __webpack_require__.t(id, 1));
      }
      function webpackAsyncContextResolve(req) {
        return Promise.resolve().then(() => {
          if (!__webpack_require__.o(map, req)) {
            var e = new Error("Cannot find module '" + req + "'");
            throw ((e.code = 'MODULE_NOT_FOUND'), e);
          }
          return map[req];
        });
      }
      (webpackAsyncContext.keys = () => Object.keys(map)),
        (webpackAsyncContext.resolve = webpackAsyncContextResolve),
        (webpackAsyncContext.id = './src/assets eager recursive ^\\.\\/.*$'),
        (module.exports = webpackAsyncContext);
    },
    './src/assets eager recursive ^\\.\\/resources.*$': (module) => {
      function webpackEmptyAsyncContext(req) {
        return Promise.resolve().then(() => {
          var e = new Error("Cannot find module '" + req + "'");
          throw ((e.code = 'MODULE_NOT_FOUND'), e);
        });
      }
      (webpackEmptyAsyncContext.keys = () => []),
        (webpackEmptyAsyncContext.resolve = webpackEmptyAsyncContext),
        (webpackEmptyAsyncContext.id = './src/assets eager recursive ^\\.\\/resources.*$'),
        (module.exports = webpackEmptyAsyncContext);
    },
    '../react-components/node_modules/date-fns/locale ../../node_modules/.pnpm/node_modules/date-fns/locale lazy recursive ^\\.\\/.*\\/index\\.js$':
      (module, __unused_webpack_exports, __webpack_require__) => {
        var map = {
          './_lib/buildFormatLongFn/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildFormatLongFn/index.js',
            7,
            45585,
          ],
          './_lib/buildLocalizeFn/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildLocalizeFn/index.js',
            7,
            69107,
          ],
          './_lib/buildMatchFn/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildMatchFn/index.js',
            7,
            95565,
          ],
          './_lib/buildMatchPatternFn/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildMatchPatternFn/index.js',
            7,
            79489,
          ],
          './af/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/af/_lib/formatDistance/index.js',
            7,
            83654,
          ],
          './af/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/af/_lib/formatLong/index.js',
            7,
            63623,
          ],
          './af/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/af/_lib/formatRelative/index.js',
            7,
            97085,
          ],
          './af/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/af/_lib/localize/index.js',
            7,
            92429,
          ],
          './af/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/af/_lib/match/index.js',
            7,
            44851,
          ],
          './af/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/af/index.js',
            7,
            45864,
          ],
          './ar-DZ/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-DZ/_lib/formatDistance/index.js',
            7,
            63399,
          ],
          './ar-DZ/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-DZ/_lib/formatLong/index.js',
            7,
            98690,
          ],
          './ar-DZ/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-DZ/_lib/formatRelative/index.js',
            7,
            69656,
          ],
          './ar-DZ/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-DZ/_lib/localize/index.js',
            7,
            36648,
          ],
          './ar-DZ/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-DZ/_lib/match/index.js',
            7,
            59824,
          ],
          './ar-DZ/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-DZ/index.js',
            7,
            11357,
          ],
          './ar-EG/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-EG/_lib/formatDistance/index.js',
            7,
            10253,
          ],
          './ar-EG/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-EG/_lib/formatLong/index.js',
            7,
            38312,
          ],
          './ar-EG/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-EG/_lib/formatRelative/index.js',
            7,
            11178,
          ],
          './ar-EG/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-EG/_lib/localize/index.js',
            7,
            96338,
          ],
          './ar-EG/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-EG/_lib/match/index.js',
            7,
            87766,
          ],
          './ar-EG/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-EG/index.js',
            7,
            86171,
          ],
          './ar-MA/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-MA/_lib/formatDistance/index.js',
            7,
            36259,
          ],
          './ar-MA/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-MA/_lib/formatLong/index.js',
            7,
            42126,
          ],
          './ar-MA/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-MA/_lib/formatRelative/index.js',
            7,
            16188,
          ],
          './ar-MA/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-MA/_lib/localize/index.js',
            7,
            98084,
          ],
          './ar-MA/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-MA/_lib/match/index.js',
            7,
            6340,
          ],
          './ar-MA/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-MA/index.js',
            7,
            74761,
          ],
          './ar-SA/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-SA/_lib/formatDistance/index.js',
            7,
            537,
          ],
          './ar-SA/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-SA/_lib/formatLong/index.js',
            7,
            96052,
          ],
          './ar-SA/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-SA/_lib/formatRelative/index.js',
            7,
            19454,
          ],
          './ar-SA/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-SA/_lib/localize/index.js',
            7,
            3774,
          ],
          './ar-SA/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-SA/_lib/match/index.js',
            7,
            68026,
          ],
          './ar-SA/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-SA/index.js',
            7,
            17303,
          ],
          './ar-TN/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-TN/_lib/formatDistance/index.js',
            7,
            73867,
          ],
          './ar-TN/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-TN/_lib/formatLong/index.js',
            7,
            34086,
          ],
          './ar-TN/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-TN/_lib/formatRelative/index.js',
            7,
            15780,
          ],
          './ar-TN/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-TN/_lib/localize/index.js',
            7,
            77212,
          ],
          './ar-TN/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-TN/_lib/match/index.js',
            7,
            95580,
          ],
          './ar-TN/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-TN/index.js',
            7,
            31201,
          ],
          './ar/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar/_lib/formatDistance/index.js',
            7,
            50882,
          ],
          './ar/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar/_lib/formatLong/index.js',
            7,
            44467,
          ],
          './ar/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar/_lib/formatRelative/index.js',
            7,
            53569,
          ],
          './ar/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar/_lib/localize/index.js',
            7,
            34697,
          ],
          './ar/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar/_lib/match/index.js',
            7,
            4935,
          ],
          './ar/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar/index.js',
            7,
            37508,
          ],
          './az/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/az/_lib/formatDistance/index.js',
            7,
            35114,
          ],
          './az/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/az/_lib/formatLong/index.js',
            7,
            37099,
          ],
          './az/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/az/_lib/formatRelative/index.js',
            7,
            91785,
          ],
          './az/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/az/_lib/localize/index.js',
            7,
            737,
          ],
          './az/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/az/_lib/match/index.js',
            7,
            79423,
          ],
          './az/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/az/index.js',
            7,
            23964,
          ],
          './be-tarask/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be-tarask/_lib/formatDistance/index.js',
            7,
            53583,
          ],
          './be-tarask/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be-tarask/_lib/formatLong/index.js',
            7,
            93050,
          ],
          './be-tarask/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be-tarask/_lib/formatRelative/index.js',
            7,
            6961,
            17088,
          ],
          './be-tarask/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be-tarask/_lib/localize/index.js',
            7,
            41408,
          ],
          './be-tarask/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be-tarask/_lib/match/index.js',
            7,
            67304,
          ],
          './be-tarask/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be-tarask/index.js',
            7,
            6961,
            21525,
          ],
          './be/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be/_lib/formatDistance/index.js',
            7,
            388,
          ],
          './be/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be/_lib/formatLong/index.js',
            7,
            25973,
          ],
          './be/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be/_lib/formatRelative/index.js',
            7,
            6961,
            80951,
          ],
          './be/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be/_lib/localize/index.js',
            7,
            29143,
          ],
          './be/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be/_lib/match/index.js',
            7,
            55481,
          ],
          './be/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be/index.js',
            7,
            6961,
            69742,
          ],
          './bg/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bg/_lib/formatDistance/index.js',
            7,
            55418,
          ],
          './bg/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bg/_lib/formatLong/index.js',
            7,
            87643,
          ],
          './bg/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bg/_lib/formatRelative/index.js',
            7,
            18105,
          ],
          './bg/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bg/_lib/localize/index.js',
            7,
            15409,
          ],
          './bg/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bg/_lib/match/index.js',
            7,
            94063,
          ],
          './bg/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bg/index.js',
            7,
            98348,
          ],
          './bn/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bn/_lib/formatDistance/index.js',
            7,
            38157,
          ],
          './bn/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bn/_lib/formatLong/index.js',
            7,
            93448,
          ],
          './bn/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bn/_lib/formatRelative/index.js',
            7,
            59114,
          ],
          './bn/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bn/_lib/localize/index.js',
            9,
            30002,
          ],
          './bn/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bn/_lib/match/index.js',
            7,
            63606,
          ],
          './bn/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bn/index.js',
            7,
            62779,
          ],
          './bs/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bs/_lib/formatDistance/index.js',
            7,
            1214,
          ],
          './bs/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bs/_lib/formatLong/index.js',
            7,
            62927,
          ],
          './bs/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bs/_lib/formatRelative/index.js',
            7,
            36469,
          ],
          './bs/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bs/_lib/localize/index.js',
            7,
            19989,
          ],
          './bs/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bs/_lib/match/index.js',
            7,
            99035,
          ],
          './bs/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bs/index.js',
            7,
            9136,
          ],
          './ca/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ca/_lib/formatDistance/index.js',
            7,
            72537,
          ],
          './ca/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ca/_lib/formatLong/index.js',
            7,
            58164,
          ],
          './ca/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ca/_lib/formatRelative/index.js',
            7,
            27934,
          ],
          './ca/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ca/_lib/localize/index.js',
            7,
            27294,
          ],
          './ca/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ca/_lib/match/index.js',
            7,
            28154,
          ],
          './ca/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ca/index.js',
            7,
            8599,
          ],
          './cs/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cs/_lib/formatDistance/index.js',
            7,
            26867,
          ],
          './cs/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cs/_lib/formatLong/index.js',
            7,
            95614,
          ],
          './cs/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cs/_lib/formatRelative/index.js',
            7,
            65004,
          ],
          './cs/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cs/_lib/localize/index.js',
            7,
            31316,
          ],
          './cs/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cs/_lib/match/index.js',
            7,
            26228,
          ],
          './cs/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cs/index.js',
            7,
            41081,
          ],
          './cy/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cy/_lib/formatDistance/index.js',
            7,
            50721,
          ],
          './cy/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cy/_lib/formatLong/index.js',
            7,
            83084,
          ],
          './cy/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cy/_lib/formatRelative/index.js',
            7,
            80582,
          ],
          './cy/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cy/_lib/localize/index.js',
            7,
            5558,
          ],
          './cy/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cy/_lib/match/index.js',
            7,
            92658,
          ],
          './cy/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cy/index.js',
            7,
            43983,
          ],
          './da/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/da/_lib/formatDistance/index.js',
            7,
            4186,
          ],
          './da/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/da/_lib/formatLong/index.js',
            7,
            86427,
          ],
          './da/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/da/_lib/formatRelative/index.js',
            7,
            14297,
          ],
          './da/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/da/_lib/localize/index.js',
            7,
            74449,
          ],
          './da/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/da/_lib/match/index.js',
            7,
            9679,
          ],
          './da/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/da/index.js',
            7,
            2092,
          ],
          './de-AT/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/de-AT/_lib/localize/index.js',
            7,
            90999,
          ],
          './de-AT/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/de-AT/index.js',
            7,
            20430,
          ],
          './de/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/de/_lib/formatDistance/index.js',
            7,
            43246,
          ],
          './de/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/de/_lib/formatLong/index.js',
            7,
            86623,
          ],
          './de/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/de/_lib/formatRelative/index.js',
            7,
            52581,
          ],
          './de/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/de/_lib/localize/index.js',
            7,
            6789,
          ],
          './de/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/de/_lib/match/index.js',
            7,
            65355,
          ],
          './de/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/de/index.js',
            7,
            95968,
          ],
          './el/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/el/_lib/formatDistance/index.js',
            7,
            19156,
          ],
          './el/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/el/_lib/formatLong/index.js',
            7,
            2053,
          ],
          './el/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/el/_lib/formatRelative/index.js',
            7,
            32935,
          ],
          './el/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/el/_lib/localize/index.js',
            7,
            61063,
          ],
          './el/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/el/_lib/match/index.js',
            7,
            73449,
          ],
          './el/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/el/index.js',
            7,
            1150,
          ],
          './en-AU/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-AU/_lib/formatLong/index.js',
            7,
            65470,
          ],
          './en-AU/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-AU/index.js',
            7,
            13945,
          ],
          './en-CA/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-CA/_lib/formatDistance/index.js',
            7,
            35793,
          ],
          './en-CA/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-CA/_lib/formatLong/index.js',
            7,
            97628,
          ],
          './en-CA/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-CA/index.js',
            7,
            53279,
          ],
          './en-GB/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-GB/_lib/formatLong/index.js',
            7,
            63701,
          ],
          './en-GB/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-GB/index.js',
            7,
            55726,
          ],
          './en-IE/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-IE/index.js',
            7,
            43265,
          ],
          './en-IN/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-IN/_lib/formatLong/index.js',
            7,
            18111,
          ],
          './en-IN/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-IN/index.js',
            7,
            68288,
          ],
          './en-NZ/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-NZ/_lib/formatLong/index.js',
            7,
            97808,
          ],
          './en-NZ/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-NZ/index.js',
            7,
            6819,
          ],
          './en-US/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-US/_lib/formatDistance/index.js',
            7,
            41993,
          ],
          './en-US/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-US/_lib/formatLong/index.js',
            7,
            99140,
          ],
          './en-US/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-US/_lib/formatRelative/index.js',
            7,
            5934,
          ],
          './en-US/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-US/_lib/localize/index.js',
            7,
            37518,
          ],
          './en-US/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-US/_lib/match/index.js',
            7,
            9514,
          ],
          './en-US/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-US/index.js',
            7,
            22279,
          ],
          './en-ZA/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-ZA/_lib/formatLong/index.js',
            7,
            47369,
          ],
          './en-ZA/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-ZA/index.js',
            7,
            59794,
          ],
          './eo/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/eo/_lib/formatDistance/index.js',
            7,
            34213,
          ],
          './eo/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/eo/_lib/formatLong/index.js',
            7,
            80016,
          ],
          './eo/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/eo/_lib/formatRelative/index.js',
            7,
            70946,
          ],
          './eo/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/eo/_lib/localize/index.js',
            7,
            89786,
          ],
          './eo/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/eo/_lib/match/index.js',
            7,
            99198,
          ],
          './eo/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/eo/index.js',
            7,
            24227,
          ],
          './es/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/es/_lib/formatDistance/index.js',
            7,
            76257,
          ],
          './es/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/es/_lib/formatLong/index.js',
            7,
            11276,
          ],
          './es/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/es/_lib/formatRelative/index.js',
            7,
            91142,
          ],
          './es/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/es/_lib/localize/index.js',
            7,
            63094,
          ],
          './es/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/es/_lib/match/index.js',
            7,
            54834,
          ],
          './es/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/es/index.js',
            7,
            14479,
          ],
          './et/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/et/_lib/formatDistance/index.js',
            7,
            49436,
          ],
          './et/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/et/_lib/formatLong/index.js',
            7,
            92445,
          ],
          './et/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/et/_lib/formatRelative/index.js',
            7,
            11663,
          ],
          './et/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/et/_lib/localize/index.js',
            7,
            33023,
          ],
          './et/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/et/_lib/match/index.js',
            7,
            42209,
          ],
          './et/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/et/index.js',
            7,
            72598,
          ],
          './eu/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/eu/_lib/formatDistance/index.js',
            7,
            59847,
          ],
          './eu/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/eu/_lib/formatLong/index.js',
            7,
            98978,
          ],
          './eu/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/eu/_lib/formatRelative/index.js',
            7,
            568,
          ],
          './eu/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/eu/_lib/localize/index.js',
            7,
            17896,
          ],
          './eu/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/eu/_lib/match/index.js',
            7,
            22768,
          ],
          './eu/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/eu/index.js',
            7,
            37693,
          ],
          './fa-IR/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fa-IR/_lib/formatDistance/index.js',
            7,
            82868,
          ],
          './fa-IR/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fa-IR/_lib/formatLong/index.js',
            7,
            53797,
          ],
          './fa-IR/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fa-IR/_lib/formatRelative/index.js',
            7,
            96647,
          ],
          './fa-IR/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fa-IR/_lib/localize/index.js',
            7,
            92135,
          ],
          './fa-IR/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fa-IR/_lib/match/index.js',
            7,
            73065,
          ],
          './fa-IR/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fa-IR/index.js',
            7,
            11614,
          ],
          './fi/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fi/_lib/formatDistance/index.js',
            7,
            65316,
          ],
          './fi/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fi/_lib/formatLong/index.js',
            7,
            85589,
          ],
          './fi/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fi/_lib/formatRelative/index.js',
            7,
            14327,
          ],
          './fi/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fi/_lib/localize/index.js',
            7,
            3447,
          ],
          './fi/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fi/_lib/match/index.js',
            7,
            46297,
          ],
          './fi/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fi/index.js',
            7,
            70926,
          ],
          './fr-CA/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fr-CA/_lib/formatLong/index.js',
            7,
            93045,
          ],
          './fr-CA/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fr-CA/index.js',
            7,
            30830,
          ],
          './fr-CH/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fr-CH/_lib/formatLong/index.js',
            7,
            62754,
          ],
          './fr-CH/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fr-CH/_lib/formatRelative/index.js',
            7,
            69208,
          ],
          './fr-CH/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fr-CH/index.js',
            7,
            97373,
          ],
          './fr/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fr/_lib/formatDistance/index.js',
            7,
            38973,
          ],
          './fr/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fr/_lib/formatLong/index.js',
            7,
            35640,
          ],
          './fr/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fr/_lib/formatRelative/index.js',
            7,
            52346,
          ],
          './fr/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fr/_lib/localize/index.js',
            7,
            41794,
          ],
          './fr/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fr/_lib/match/index.js',
            7,
            51622,
          ],
          './fr/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fr/index.js',
            7,
            9515,
          ],
          './fy/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fy/_lib/formatDistance/index.js',
            7,
            660,
          ],
          './fy/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fy/_lib/formatLong/index.js',
            7,
            86085,
          ],
          './fy/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fy/_lib/formatRelative/index.js',
            7,
            97767,
          ],
          './fy/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fy/_lib/localize/index.js',
            7,
            88775,
          ],
          './fy/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fy/_lib/match/index.js',
            7,
            87337,
          ],
          './fy/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fy/index.js',
            7,
            7614,
          ],
          './gd/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gd/_lib/formatDistance/index.js',
            7,
            82742,
          ],
          './gd/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gd/_lib/formatLong/index.js',
            7,
            72343,
          ],
          './gd/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gd/_lib/formatRelative/index.js',
            7,
            60173,
          ],
          './gd/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gd/_lib/localize/index.js',
            7,
            68125,
          ],
          './gd/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gd/_lib/match/index.js',
            7,
            91459,
          ],
          './gd/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gd/index.js',
            7,
            72472,
          ],
          './gl/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gl/_lib/formatDistance/index.js',
            7,
            36478,
          ],
          './gl/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gl/_lib/formatLong/index.js',
            7,
            67407,
          ],
          './gl/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gl/_lib/formatRelative/index.js',
            7,
            9493,
          ],
          './gl/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gl/_lib/localize/index.js',
            7,
            3829,
          ],
          './gl/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gl/_lib/match/index.js',
            7,
            82651,
          ],
          './gl/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gl/index.js',
            7,
            44784,
          ],
          './gu/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gu/_lib/formatDistance/index.js',
            7,
            15617,
          ],
          './gu/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gu/_lib/formatLong/index.js',
            7,
            24172,
          ],
          './gu/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gu/_lib/formatRelative/index.js',
            7,
            75430,
          ],
          './gu/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gu/_lib/localize/index.js',
            7,
            90262,
          ],
          './gu/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gu/_lib/match/index.js',
            7,
            54898,
          ],
          './gu/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gu/index.js',
            7,
            27023,
          ],
          './he/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/he/_lib/formatDistance/index.js',
            7,
            3546,
          ],
          './he/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/he/_lib/formatLong/index.js',
            7,
            92411,
          ],
          './he/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/he/_lib/formatRelative/index.js',
            7,
            10937,
          ],
          './he/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/he/_lib/localize/index.js',
            7,
            43057,
          ],
          './he/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/he/_lib/match/index.js',
            7,
            65135,
          ],
          './he/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/he/index.js',
            7,
            73964,
          ],
          './hi/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hi/_lib/formatDistance/index.js',
            7,
            60454,
          ],
          './hi/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hi/_lib/formatLong/index.js',
            7,
            14183,
          ],
          './hi/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hi/_lib/formatRelative/index.js',
            7,
            61789,
          ],
          './hi/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hi/_lib/localize/index.js',
            9,
            69421,
          ],
          './hi/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hi/_lib/match/index.js',
            7,
            82067,
          ],
          './hi/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hi/index.js',
            7,
            85928,
          ],
          './hr/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hr/_lib/formatDistance/index.js',
            7,
            11683,
          ],
          './hr/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hr/_lib/formatLong/index.js',
            7,
            90414,
          ],
          './hr/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hr/_lib/formatRelative/index.js',
            7,
            65148,
          ],
          './hr/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hr/_lib/localize/index.js',
            7,
            25156,
          ],
          './hr/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hr/_lib/match/index.js',
            7,
            31844,
          ],
          './hr/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hr/index.js',
            7,
            87209,
          ],
          './ht/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ht/_lib/formatDistance/index.js',
            7,
            76197,
          ],
          './ht/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ht/_lib/formatLong/index.js',
            7,
            56784,
          ],
          './ht/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ht/_lib/formatRelative/index.js',
            7,
            610,
          ],
          './ht/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ht/_lib/localize/index.js',
            7,
            18618,
          ],
          './ht/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ht/_lib/match/index.js',
            7,
            27070,
          ],
          './ht/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ht/index.js',
            7,
            21315,
          ],
          './hu/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hu/_lib/formatDistance/index.js',
            7,
            95338,
          ],
          './hu/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hu/_lib/formatLong/index.js',
            7,
            73579,
          ],
          './hu/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hu/_lib/formatRelative/index.js',
            7,
            43593,
          ],
          './hu/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hu/_lib/localize/index.js',
            7,
            50049,
          ],
          './hu/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hu/_lib/match/index.js',
            7,
            5887,
          ],
          './hu/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hu/index.js',
            7,
            71740,
          ],
          './hy/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hy/_lib/formatDistance/index.js',
            7,
            44726,
          ],
          './hy/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hy/_lib/formatLong/index.js',
            7,
            60087,
          ],
          './hy/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hy/_lib/formatRelative/index.js',
            7,
            38509,
          ],
          './hy/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hy/_lib/localize/index.js',
            7,
            75261,
          ],
          './hy/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hy/_lib/match/index.js',
            7,
            61315,
          ],
          './hy/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hy/index.js',
            7,
            55416,
          ],
          './id/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/id/_lib/formatDistance/index.js',
            7,
            77640,
          ],
          './id/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/id/_lib/formatLong/index.js',
            7,
            69257,
          ],
          './id/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/id/_lib/formatRelative/index.js',
            7,
            52323,
          ],
          './id/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/id/_lib/localize/index.js',
            7,
            62347,
          ],
          './id/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/id/_lib/match/index.js',
            7,
            92597,
          ],
          './id/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/id/index.js',
            7,
            74994,
          ],
          './is/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/is/_lib/formatDistance/index.js',
            7,
            33237,
          ],
          './is/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/is/_lib/formatLong/index.js',
            7,
            13888,
          ],
          './is/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/is/_lib/formatRelative/index.js',
            7,
            42770,
          ],
          './is/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/is/_lib/localize/index.js',
            7,
            97610,
          ],
          './is/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/is/_lib/match/index.js',
            7,
            19662,
          ],
          './is/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/is/index.js',
            7,
            84339,
          ],
          './it-CH/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/it-CH/_lib/formatLong/index.js',
            7,
            82533,
          ],
          './it-CH/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/it-CH/index.js',
            7,
            33118,
          ],
          './it/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/it/_lib/formatDistance/index.js',
            7,
            90712,
          ],
          './it/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/it/_lib/formatLong/index.js',
            7,
            89177,
          ],
          './it/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/it/_lib/formatRelative/index.js',
            7,
            46323,
          ],
          './it/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/it/_lib/localize/index.js',
            7,
            19419,
          ],
          './it/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/it/_lib/match/index.js',
            7,
            92965,
          ],
          './it/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/it/index.js',
            7,
            95746,
          ],
          './ja-Hira/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ja-Hira/_lib/formatDistance/index.js',
            7,
            48573,
          ],
          './ja-Hira/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ja-Hira/_lib/formatLong/index.js',
            7,
            7032,
          ],
          './ja-Hira/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ja-Hira/_lib/formatRelative/index.js',
            7,
            10746,
          ],
          './ja-Hira/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ja-Hira/_lib/localize/index.js',
            7,
            4514,
          ],
          './ja-Hira/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ja-Hira/_lib/match/index.js',
            7,
            46342,
          ],
          './ja-Hira/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ja-Hira/index.js',
            7,
            65579,
          ],
          './ja/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ja/_lib/formatDistance/index.js',
            7,
            19728,
          ],
          './ja/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ja/_lib/formatLong/index.js',
            7,
            88129,
          ],
          './ja/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ja/_lib/formatRelative/index.js',
            7,
            96395,
          ],
          './ja/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ja/_lib/localize/index.js',
            7,
            86915,
          ],
          './ja/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ja/_lib/match/index.js',
            7,
            26541,
          ],
          './ja/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ja/index.js',
            7,
            89706,
          ],
          './ka/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ka/_lib/formatDistance/index.js',
            7,
            93521,
          ],
          './ka/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ka/_lib/formatLong/index.js',
            7,
            84252,
          ],
          './ka/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ka/_lib/formatRelative/index.js',
            7,
            84950,
          ],
          './ka/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ka/_lib/localize/index.js',
            7,
            86534,
          ],
          './ka/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ka/_lib/match/index.js',
            7,
            70210,
          ],
          './ka/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ka/index.js',
            7,
            55871,
          ],
          './kk/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kk/_lib/formatDistance/index.js',
            7,
            85987,
          ],
          './kk/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kk/_lib/formatLong/index.js',
            7,
            3598,
          ],
          './kk/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kk/_lib/formatRelative/index.js',
            7,
            1052,
          ],
          './kk/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kk/_lib/localize/index.js',
            7,
            61284,
          ],
          './kk/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kk/_lib/match/index.js',
            7,
            81380,
          ],
          './kk/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kk/index.js',
            7,
            23433,
          ],
          './km/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/km/_lib/formatDistance/index.js',
            7,
            11941,
          ],
          './km/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/km/_lib/formatLong/index.js',
            7,
            50736,
          ],
          './km/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/km/_lib/formatRelative/index.js',
            7,
            36226,
          ],
          './km/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/km/_lib/localize/index.js',
            7,
            28634,
          ],
          './km/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/km/_lib/match/index.js',
            7,
            70686,
          ],
          './km/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/km/index.js',
            7,
            19843,
          ],
          './kn/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kn/_lib/formatDistance/index.js',
            7,
            28020,
          ],
          './kn/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kn/_lib/formatLong/index.js',
            7,
            74181,
          ],
          './kn/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kn/_lib/formatRelative/index.js',
            7,
            12583,
          ],
          './kn/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kn/_lib/localize/index.js',
            7,
            50023,
          ],
          './kn/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kn/_lib/match/index.js',
            7,
            40425,
          ],
          './kn/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kn/index.js',
            7,
            70142,
          ],
          './ko/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ko/_lib/formatDistance/index.js',
            7,
            85311,
          ],
          './ko/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ko/_lib/formatLong/index.js',
            7,
            92330,
          ],
          './ko/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ko/_lib/formatRelative/index.js',
            7,
            9264,
          ],
          './ko/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ko/_lib/localize/index.js',
            7,
            92752,
          ],
          './ko/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ko/_lib/match/index.js',
            7,
            10520,
          ],
          './ko/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ko/index.js',
            7,
            47237,
          ],
          './lb/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/lb/_lib/formatDistance/index.js',
            7,
            60519,
          ],
          './lb/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/lb/_lib/formatLong/index.js',
            7,
            45026,
          ],
          './lb/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/lb/_lib/formatRelative/index.js',
            7,
            66776,
          ],
          './lb/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/lb/_lib/localize/index.js',
            7,
            19880,
          ],
          './lb/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/lb/_lib/match/index.js',
            7,
            66448,
          ],
          './lb/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/lb/index.js',
            7,
            68989,
          ],
          './lt/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/lt/_lib/formatDistance/index.js',
            7,
            46481,
          ],
          './lt/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/lt/_lib/formatLong/index.js',
            7,
            23260,
          ],
          './lt/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/lt/_lib/formatRelative/index.js',
            7,
            95222,
          ],
          './lt/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/lt/_lib/localize/index.js',
            7,
            19238,
          ],
          './lt/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/lt/_lib/match/index.js',
            7,
            8418,
          ],
          './lt/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/lt/index.js',
            7,
            21503,
          ],
          './lv/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/lv/_lib/formatDistance/index.js',
            7,
            52699,
          ],
          './lv/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/lv/_lib/formatLong/index.js',
            7,
            15062,
          ],
          './lv/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/lv/_lib/formatRelative/index.js',
            7,
            14036,
          ],
          './lv/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/lv/_lib/localize/index.js',
            7,
            20012,
          ],
          './lv/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/lv/_lib/match/index.js',
            7,
            60620,
          ],
          './lv/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/lv/index.js',
            7,
            2929,
          ],
          './mk/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mk/_lib/formatDistance/index.js',
            7,
            79425,
          ],
          './mk/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mk/_lib/formatLong/index.js',
            7,
            75756,
          ],
          './mk/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mk/_lib/formatRelative/index.js',
            7,
            94310,
          ],
          './mk/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mk/_lib/localize/index.js',
            7,
            70870,
          ],
          './mk/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mk/_lib/match/index.js',
            7,
            26450,
          ],
          './mk/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mk/index.js',
            7,
            53423,
          ],
          './mn/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mn/_lib/formatDistance/index.js',
            7,
            66898,
          ],
          './mn/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mn/_lib/formatLong/index.js',
            7,
            63683,
          ],
          './mn/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mn/_lib/formatRelative/index.js',
            7,
            65457,
          ],
          './mn/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mn/_lib/localize/index.js',
            7,
            44281,
          ],
          './mn/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mn/_lib/match/index.js',
            7,
            2167,
          ],
          './mn/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mn/index.js',
            7,
            17620,
          ],
          './ms/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ms/_lib/formatDistance/index.js',
            7,
            78777,
          ],
          './ms/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ms/_lib/formatLong/index.js',
            7,
            68916,
          ],
          './ms/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ms/_lib/formatRelative/index.js',
            7,
            59038,
          ],
          './ms/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ms/_lib/localize/index.js',
            7,
            33534,
          ],
          './ms/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ms/_lib/match/index.js',
            7,
            47098,
          ],
          './ms/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ms/index.js',
            7,
            43895,
          ],
          './mt/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mt/_lib/formatDistance/index.js',
            7,
            22324,
          ],
          './mt/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mt/_lib/formatLong/index.js',
            7,
            82885,
          ],
          './mt/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mt/_lib/formatRelative/index.js',
            7,
            36103,
          ],
          './mt/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mt/_lib/localize/index.js',
            7,
            68839,
          ],
          './mt/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mt/_lib/match/index.js',
            7,
            45065,
          ],
          './mt/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mt/index.js',
            7,
            40094,
          ],
          './nb/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nb/_lib/formatDistance/index.js',
            7,
            9381,
          ],
          './nb/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nb/_lib/formatLong/index.js',
            7,
            86928,
          ],
          './nb/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nb/_lib/formatRelative/index.js',
            7,
            46114,
          ],
          './nb/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nb/_lib/localize/index.js',
            7,
            89146,
          ],
          './nb/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nb/_lib/match/index.js',
            7,
            97438,
          ],
          './nb/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nb/index.js',
            7,
            68227,
          ],
          './nl-BE/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nl-BE/_lib/formatDistance/index.js',
            7,
            44799,
          ],
          './nl-BE/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nl-BE/_lib/formatLong/index.js',
            7,
            59242,
          ],
          './nl-BE/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nl-BE/_lib/formatRelative/index.js',
            7,
            68336,
          ],
          './nl-BE/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nl-BE/_lib/localize/index.js',
            7,
            46608,
          ],
          './nl-BE/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nl-BE/_lib/match/index.js',
            7,
            7096,
          ],
          './nl-BE/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nl-BE/index.js',
            7,
            41669,
          ],
          './nl/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nl/_lib/formatDistance/index.js',
            7,
            18363,
          ],
          './nl/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nl/_lib/formatLong/index.js',
            7,
            9910,
          ],
          './nl/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nl/_lib/formatRelative/index.js',
            7,
            4436,
          ],
          './nl/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nl/_lib/localize/index.js',
            7,
            71500,
          ],
          './nl/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nl/_lib/match/index.js',
            7,
            51308,
          ],
          './nl/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nl/index.js',
            7,
            96049,
          ],
          './nn/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nn/_lib/formatDistance/index.js',
            7,
            62161,
          ],
          './nn/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nn/_lib/formatLong/index.js',
            7,
            66140,
          ],
          './nn/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nn/_lib/formatRelative/index.js',
            7,
            62998,
          ],
          './nn/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nn/_lib/localize/index.js',
            7,
            68902,
          ],
          './nn/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nn/_lib/match/index.js',
            7,
            89378,
          ],
          './nn/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nn/index.js',
            7,
            64607,
          ],
          './oc/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/oc/_lib/formatDistance/index.js',
            7,
            62127,
          ],
          './oc/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/oc/_lib/formatLong/index.js',
            7,
            10906,
          ],
          './oc/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/oc/_lib/formatRelative/index.js',
            7,
            41504,
          ],
          './oc/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/oc/_lib/localize/index.js',
            7,
            18048,
          ],
          './oc/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/oc/_lib/match/index.js',
            7,
            35624,
          ],
          './oc/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/oc/index.js',
            7,
            93493,
          ],
          './pl/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pl/_lib/formatDistance/index.js',
            7,
            94133,
          ],
          './pl/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pl/_lib/formatLong/index.js',
            7,
            92320,
          ],
          './pl/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pl/_lib/formatRelative/index.js',
            7,
            23218,
          ],
          './pl/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pl/_lib/localize/index.js',
            7,
            43690,
          ],
          './pl/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pl/_lib/match/index.js',
            7,
            62894,
          ],
          './pl/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pl/index.js',
            7,
            58675,
          ],
          './pt-BR/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt-BR/_lib/formatDistance/index.js',
            7,
            8036,
          ],
          './pt-BR/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt-BR/_lib/formatLong/index.js',
            7,
            5845,
          ],
          './pt-BR/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt-BR/_lib/formatRelative/index.js',
            7,
            66423,
          ],
          './pt-BR/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt-BR/_lib/localize/index.js',
            7,
            19607,
          ],
          './pt-BR/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt-BR/_lib/match/index.js',
            7,
            39001,
          ],
          './pt-BR/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt-BR/index.js',
            7,
            76910,
          ],
          './pt/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt/_lib/formatDistance/index.js',
            7,
            37821,
          ],
          './pt/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt/_lib/formatLong/index.js',
            7,
            42552,
          ],
          './pt/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt/_lib/formatRelative/index.js',
            7,
            79226,
          ],
          './pt/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt/_lib/localize/index.js',
            7,
            17282,
          ],
          './pt/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt/_lib/match/index.js',
            7,
            65222,
          ],
          './pt/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt/index.js',
            7,
            62827,
          ],
          './ro/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ro/_lib/formatDistance/index.js',
            7,
            61250,
          ],
          './ro/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ro/_lib/formatLong/index.js',
            7,
            2579,
          ],
          './ro/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ro/_lib/formatRelative/index.js',
            7,
            48129,
          ],
          './ro/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ro/_lib/localize/index.js',
            7,
            56233,
          ],
          './ro/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ro/_lib/match/index.js',
            7,
            45191,
          ],
          './ro/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ro/index.js',
            7,
            55684,
          ],
          './ru/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ru/_lib/formatDistance/index.js',
            7,
            13764,
          ],
          './ru/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ru/_lib/formatLong/index.js',
            7,
            65109,
          ],
          './ru/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ru/_lib/formatRelative/index.js',
            7,
            79127,
          ],
          './ru/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ru/_lib/localize/index.js',
            7,
            9463,
          ],
          './ru/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ru/_lib/match/index.js',
            7,
            67641,
          ],
          './ru/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ru/index.js',
            7,
            75278,
          ],
          './sk/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sk/_lib/formatDistance/index.js',
            7,
            13355,
          ],
          './sk/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sk/_lib/formatLong/index.js',
            7,
            26950,
          ],
          './sk/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sk/_lib/formatRelative/index.js',
            7,
            88484,
          ],
          './sk/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sk/_lib/localize/index.js',
            7,
            37756,
          ],
          './sk/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sk/_lib/match/index.js',
            7,
            94492,
          ],
          './sk/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sk/index.js',
            7,
            24961,
          ],
          './sl/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sl/_lib/formatDistance/index.js',
            7,
            44914,
          ],
          './sl/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sl/_lib/formatLong/index.js',
            7,
            26243,
          ],
          './sl/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sl/_lib/formatRelative/index.js',
            7,
            49041,
          ],
          './sl/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sl/_lib/localize/index.js',
            7,
            1369,
          ],
          './sl/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sl/_lib/match/index.js',
            7,
            87415,
          ],
          './sl/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sl/index.js',
            7,
            50932,
          ],
          './sq/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sq/_lib/formatDistance/index.js',
            7,
            23513,
          ],
          './sq/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sq/_lib/formatLong/index.js',
            7,
            52180,
          ],
          './sq/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sq/_lib/formatRelative/index.js',
            7,
            42430,
          ],
          './sq/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sq/_lib/localize/index.js',
            7,
            10334,
          ],
          './sq/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sq/_lib/match/index.js',
            7,
            89498,
          ],
          './sq/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sq/index.js',
            7,
            91767,
          ],
          './sr-Latn/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sr-Latn/_lib/formatDistance/index.js',
            7,
            93106,
          ],
          './sr-Latn/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sr-Latn/_lib/formatLong/index.js',
            7,
            10595,
          ],
          './sr-Latn/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sr-Latn/_lib/formatRelative/index.js',
            7,
            86513,
          ],
          './sr-Latn/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sr-Latn/_lib/localize/index.js',
            7,
            20857,
          ],
          './sr-Latn/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sr-Latn/_lib/match/index.js',
            7,
            29911,
          ],
          './sr-Latn/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sr-Latn/index.js',
            7,
            35316,
          ],
          './sr/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sr/_lib/formatDistance/index.js',
            7,
            70952,
          ],
          './sr/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sr/_lib/formatLong/index.js',
            7,
            88713,
          ],
          './sr/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sr/_lib/formatRelative/index.js',
            7,
            83939,
          ],
          './sr/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sr/_lib/localize/index.js',
            7,
            3275,
          ],
          './sr/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sr/_lib/match/index.js',
            7,
            81589,
          ],
          './sr/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sr/index.js',
            7,
            86226,
          ],
          './sv/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sv/_lib/formatDistance/index.js',
            7,
            31188,
          ],
          './sv/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sv/_lib/formatLong/index.js',
            7,
            55013,
          ],
          './sv/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sv/_lib/formatRelative/index.js',
            7,
            15751,
          ],
          './sv/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sv/_lib/localize/index.js',
            7,
            57799,
          ],
          './sv/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sv/_lib/match/index.js',
            7,
            12041,
          ],
          './sv/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sv/index.js',
            7,
            9086,
          ],
          './ta/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ta/_lib/formatDistance/index.js',
            7,
            24746,
          ],
          './ta/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ta/_lib/formatLong/index.js',
            7,
            9707,
          ],
          './ta/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ta/_lib/formatRelative/index.js',
            7,
            81417,
          ],
          './ta/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ta/_lib/localize/index.js',
            7,
            7393,
          ],
          './ta/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ta/_lib/match/index.js',
            7,
            2527,
          ],
          './ta/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ta/index.js',
            7,
            668,
          ],
          './te/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/te/_lib/formatDistance/index.js',
            7,
            47902,
          ],
          './te/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/te/_lib/formatLong/index.js',
            7,
            41743,
          ],
          './te/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/te/_lib/formatRelative/index.js',
            7,
            75093,
          ],
          './te/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/te/_lib/localize/index.js',
            7,
            61781,
          ],
          './te/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/te/_lib/match/index.js',
            7,
            78971,
          ],
          './te/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/te/index.js',
            7,
            18416,
          ],
          './th/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/th/_lib/formatDistance/index.js',
            7,
            65021,
          ],
          './th/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/th/_lib/formatLong/index.js',
            7,
            39032,
          ],
          './th/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/th/_lib/formatRelative/index.js',
            7,
            27194,
          ],
          './th/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/th/_lib/localize/index.js',
            7,
            48034,
          ],
          './th/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/th/_lib/match/index.js',
            7,
            98310,
          ],
          './th/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/th/index.js',
            7,
            1195,
          ],
          './tr/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/tr/_lib/formatDistance/index.js',
            7,
            89103,
          ],
          './tr/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/tr/_lib/formatLong/index.js',
            7,
            37402,
          ],
          './tr/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/tr/_lib/formatRelative/index.js',
            7,
            36256,
          ],
          './tr/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/tr/_lib/localize/index.js',
            7,
            58912,
          ],
          './tr/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/tr/_lib/match/index.js',
            7,
            74856,
          ],
          './tr/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/tr/index.js',
            7,
            16949,
          ],
          './ug/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ug/_lib/formatDistance/index.js',
            7,
            27789,
          ],
          './ug/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ug/_lib/formatLong/index.js',
            7,
            49704,
          ],
          './ug/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ug/_lib/formatRelative/index.js',
            7,
            65098,
          ],
          './ug/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ug/_lib/localize/index.js',
            7,
            20306,
          ],
          './ug/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ug/_lib/match/index.js',
            7,
            70358,
          ],
          './ug/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ug/index.js',
            7,
            23131,
          ],
          './uk/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uk/_lib/formatDistance/index.js',
            7,
            20745,
          ],
          './uk/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uk/_lib/formatLong/index.js',
            7,
            7140,
          ],
          './uk/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uk/_lib/formatRelative/index.js',
            7,
            6961,
            64430,
          ],
          './uk/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uk/_lib/localize/index.js',
            7,
            20590,
          ],
          './uk/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uk/_lib/match/index.js',
            7,
            88490,
          ],
          './uk/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uk/index.js',
            7,
            6961,
            27591,
          ],
          './uz-Cyrl/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz-Cyrl/_lib/formatDistance/index.js',
            7,
            1491,
          ],
          './uz-Cyrl/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz-Cyrl/_lib/formatLong/index.js',
            7,
            38750,
          ],
          './uz-Cyrl/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz-Cyrl/_lib/formatRelative/index.js',
            7,
            39628,
          ],
          './uz-Cyrl/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz-Cyrl/_lib/localize/index.js',
            7,
            69364,
          ],
          './uz-Cyrl/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz-Cyrl/_lib/match/index.js',
            7,
            11156,
          ],
          './uz-Cyrl/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz-Cyrl/index.js',
            7,
            52281,
          ],
          './uz/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz/_lib/formatDistance/index.js',
            7,
            68465,
          ],
          './uz/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz/_lib/formatLong/index.js',
            7,
            35535,
          ],
          './uz/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz/_lib/formatRelative/index.js',
            7,
            26101,
          ],
          './uz/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz/_lib/localize/index.js',
            7,
            26645,
          ],
          './uz/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz/_lib/match/index.js',
            7,
            22139,
          ],
          './uz/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz/index.js',
            7,
            85840,
          ],
          './vi/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/vi/_lib/formatDistance/index.js',
            7,
            10708,
          ],
          './vi/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/vi/_lib/formatLong/index.js',
            7,
            59525,
          ],
          './vi/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/vi/_lib/formatRelative/index.js',
            7,
            64231,
          ],
          './vi/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/vi/_lib/localize/index.js',
            7,
            25223,
          ],
          './vi/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/vi/_lib/match/index.js',
            7,
            97353,
          ],
          './vi/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/vi/index.js',
            7,
            64478,
          ],
          './zh-CN/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-CN/_lib/formatDistance/index.js',
            7,
            62331,
          ],
          './zh-CN/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-CN/_lib/formatLong/index.js',
            7,
            15958,
          ],
          './zh-CN/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-CN/_lib/formatRelative/index.js',
            7,
            23668,
          ],
          './zh-CN/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-CN/_lib/localize/index.js',
            7,
            30508,
          ],
          './zh-CN/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-CN/_lib/match/index.js',
            7,
            59212,
          ],
          './zh-CN/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-CN/index.js',
            7,
            71761,
          ],
          './zh-HK/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-HK/_lib/formatDistance/index.js',
            7,
            86231,
          ],
          './zh-HK/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-HK/_lib/formatLong/index.js',
            7,
            59474,
          ],
          './zh-HK/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-HK/_lib/formatRelative/index.js',
            7,
            90344,
          ],
          './zh-HK/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-HK/_lib/localize/index.js',
            7,
            92344,
          ],
          './zh-HK/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-HK/_lib/match/index.js',
            7,
            68352,
          ],
          './zh-HK/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-HK/index.js',
            7,
            60621,
          ],
          './zh-TW/_lib/formatDistance/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-TW/_lib/formatDistance/index.js',
            7,
            17231,
          ],
          './zh-TW/_lib/formatLong/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-TW/_lib/formatLong/index.js',
            7,
            86074,
          ],
          './zh-TW/_lib/formatRelative/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-TW/_lib/formatRelative/index.js',
            7,
            80736,
          ],
          './zh-TW/_lib/localize/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-TW/_lib/localize/index.js',
            7,
            38144,
          ],
          './zh-TW/_lib/match/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-TW/_lib/match/index.js',
            7,
            31976,
          ],
          './zh-TW/index.js': [
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-TW/index.js',
            7,
            80245,
          ],
        };
        function webpackAsyncContext(req) {
          if (!__webpack_require__.o(map, req))
            return Promise.resolve().then(() => {
              var e = new Error("Cannot find module '" + req + "'");
              throw ((e.code = 'MODULE_NOT_FOUND'), e);
            });
          var ids = map[req],
            id = ids[0];
          return Promise.all(ids.slice(2).map(__webpack_require__.e)).then(() =>
            __webpack_require__.t(id, 16 | ids[1]),
          );
        }
        (webpackAsyncContext.keys = () => Object.keys(map)),
          (webpackAsyncContext.id =
            '../react-components/node_modules/date-fns/locale ../../node_modules/.pnpm/node_modules/date-fns/locale lazy recursive ^\\.\\/.*\\/index\\.js$'),
          (module.exports = webpackAsyncContext);
      },
    './src/assets/defaultLogo.png': (module, __unused_webpack_exports, __webpack_require__) => {
      'use strict';
      module.exports = __webpack_require__.p + 'static/media/defaultLogo.ce88f532.png';
    },
  },
]);
