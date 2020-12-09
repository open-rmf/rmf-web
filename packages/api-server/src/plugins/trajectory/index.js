'use strict';
var __assign =
  (this && this.__assign) ||
  function () {
    __assign =
      Object.assign ||
      function (t) {
        for (var s, i = 1, n = arguments.length; i < n; i++) {
          s = arguments[i];
          for (var p in s) if (Object.prototype.hasOwnProperty.call(s, p)) t[p] = s[p];
        }
        return t;
      };
    return __assign.apply(this, arguments);
  };
var __createBinding =
  (this && this.__createBinding) ||
  (Object.create
    ? function (o, m, k, k2) {
        if (k2 === undefined) k2 = k;
        Object.defineProperty(o, k2, {
          enumerable: true,
          get: function () {
            return m[k];
          },
        });
      }
    : function (o, m, k, k2) {
        if (k2 === undefined) k2 = k;
        o[k2] = m[k];
      });
var __setModuleDefault =
  (this && this.__setModuleDefault) ||
  (Object.create
    ? function (o, v) {
        Object.defineProperty(o, 'default', { enumerable: true, value: v });
      }
    : function (o, v) {
        o['default'] = v;
      });
var __importStar =
  (this && this.__importStar) ||
  function (mod) {
    if (mod && mod.__esModule) return mod;
    var result = {};
    if (mod != null)
      for (var k in mod)
        if (k !== 'default' && Object.prototype.hasOwnProperty.call(mod, k))
          __createBinding(result, mod, k);
    __setModuleDefault(result, mod);
    return result;
  };
var __awaiter =
  (this && this.__awaiter) ||
  function (thisArg, _arguments, P, generator) {
    function adopt(value) {
      return value instanceof P
        ? value
        : new P(function (resolve) {
            resolve(value);
          });
    }
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
          step(generator['throw'](value));
        } catch (e) {
          reject(e);
        }
      }
      function step(result) {
        result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected);
      }
      step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
  };
var __generator =
  (this && this.__generator) ||
  function (thisArg, body) {
    var _ = {
        label: 0,
        sent: function () {
          if (t[0] & 1) throw t[1];
          return t[1];
        },
        trys: [],
        ops: [],
      },
      f,
      y,
      t,
      g;
    return (
      (g = { next: verb(0), throw: verb(1), return: verb(2) }),
      typeof Symbol === 'function' &&
        (g[Symbol.iterator] = function () {
          return this;
        }),
      g
    );
    function verb(n) {
      return function (v) {
        return step([n, v]);
      };
    }
    function step(op) {
      if (f) throw new TypeError('Generator is already executing.');
      while (_)
        try {
          if (
            ((f = 1),
            y &&
              (t =
                op[0] & 2
                  ? y['return']
                  : op[0]
                  ? y['throw'] || ((t = y['return']) && t.call(y), 0)
                  : y.next) &&
              !(t = t.call(y, op[1])).done)
          )
            return t;
          if (((y = 0), t)) op = [op[0] & 2, t.value];
          switch (op[0]) {
            case 0:
            case 1:
              t = op;
              break;
            case 4:
              _.label++;
              return { value: op[1], done: false };
            case 5:
              _.label++;
              y = op[1];
              op = [0];
              continue;
            case 7:
              op = _.ops.pop();
              _.trys.pop();
              continue;
            default:
              if (
                !((t = _.trys), (t = t.length > 0 && t[t.length - 1])) &&
                (op[0] === 6 || op[0] === 2)
              ) {
                _ = 0;
                continue;
              }
              if (op[0] === 3 && (!t || (op[1] > t[0] && op[1] < t[3]))) {
                _.label = op[1];
                break;
              }
              if (op[0] === 6 && _.label < t[1]) {
                _.label = t[1];
                t = op;
                break;
              }
              if (t && _.label < t[2]) {
                _.label = t[2];
                _.ops.push(op);
                break;
              }
              if (t[2]) _.ops.pop();
              _.trys.pop();
              continue;
          }
          op = body.call(thisArg, _);
        } catch (e) {
          op = [6, e];
          y = 0;
        } finally {
          f = t = 0;
        }
      if (op[0] & 5) throw op[1];
      return { value: op[0] ? op[1] : void 0, done: true };
    }
  };
var __importDefault =
  (this && this.__importDefault) ||
  function (mod) {
    return mod && mod.__esModule ? mod : { default: mod };
  };
Object.defineProperty(exports, '__esModule', { value: true });
exports.onLoad = exports.options = void 0;
var assert = __importStar(require('assert'));
var ws_1 = __importDefault(require('ws'));
var logger_1 = __importDefault(require('../../logger'));
function options(yargs) {
  return yargs.option('trajectoryServerUrl', {
    type: 'string',
    demandOption: true,
  });
}
exports.options = options;
function onLoad(config, api) {
  return __awaiter(this, void 0, void 0, function () {
    var plugin;
    return __generator(this, function (_a) {
      plugin = new TrajectoryPlugin(config);
      api.registerHandler('latestTrajectory', function (params) {
        return plugin.latestTrajectory(params);
      });
      api.registerHandler('trajectoryServerTime', function (params) {
        return plugin.serverTime(params);
      });
      return [2 /*return*/];
    });
  });
}
exports.onLoad = onLoad;
var TrajectoryPlugin = /** @class */ (function () {
  function TrajectoryPlugin(_config) {
    this._config = _config;
    this._ongoingRequests = [];
  }
  TrajectoryPlugin.prototype.latestTrajectory = function (params) {
    return __awaiter(this, void 0, void 0, function () {
      var request, data, resp;
      var _this = this;
      return __generator(this, function (_a) {
        switch (_a.label) {
          case 0:
            request = __assign({ request: 'trajectory' }, params);
            this._send(JSON.stringify(request));
            return [
              4 /*yield*/,
              new Promise(function (res) {
                return _this._ongoingRequests.push(res);
              }),
            ];
          case 1:
            data = _a.sent();
            assert.ok(typeof data === 'string');
            resp = JSON.parse(data);
            assert.strictEqual(resp.response, 'trajectory');
            if (resp.values === null) {
              resp.values = [];
            }
            delete resp.response;
            return [2 /*return*/, resp];
        }
      });
    });
  };
  TrajectoryPlugin.prototype.serverTime = function (params) {
    return __awaiter(this, void 0, void 0, function () {
      var request, data, resp;
      var _this = this;
      return __generator(this, function (_a) {
        switch (_a.label) {
          case 0:
            request = __assign({ request: 'time' }, params);
            this._send(JSON.stringify(request));
            return [
              4 /*yield*/,
              new Promise(function (res) {
                return _this._ongoingRequests.push(res);
              }),
            ];
          case 1:
            data = _a.sent();
            assert.ok(typeof data === 'string');
            resp = JSON.parse(data);
            assert.strictEqual(resp.response, 'time');
            delete resp.response;
            return [2 /*return*/, resp];
        }
      });
    });
  };
  TrajectoryPlugin._connect = function (config) {
    return __awaiter(this, void 0, void 0, function () {
      var socket;
      return __generator(this, function (_a) {
        switch (_a.label) {
          case 0:
            logger_1.default.info(
              'connecting to trajectory server at ' + config.trajectoryServerUrl,
            );
            socket = new ws_1.default(config.trajectoryServerUrl, { handshakeTimeout: 5000 });
            return [
              4 /*yield*/,
              new Promise(function (res, rej) {
                var errorHandler = function (error) {
                  return rej(error);
                };
                socket.on('error', errorHandler);
                socket.once('open', function () {
                  socket.off('close', errorHandler);
                  res();
                });
              }),
            ];
          case 1:
            _a.sent();
            logger_1.default.info('succesfully connected to trajectory server');
            return [2 /*return*/, socket];
        }
      });
    });
  };
  TrajectoryPlugin.prototype._send = function (payload) {
    return __awaiter(this, void 0, void 0, function () {
      var _a;
      var _this = this;
      return __generator(this, function (_b) {
        switch (_b.label) {
          case 0:
            if (!!this.socket) return [3 /*break*/, 2];
            this._ongoingRequests = [];
            _a = this;
            return [4 /*yield*/, TrajectoryPlugin._connect(this._config)];
          case 1:
            _a.socket = _b.sent();
            this.socket.on('message', function (data) {
              var resolve = _this._ongoingRequests.shift();
              resolve && resolve(data);
            });
            _b.label = 2;
          case 2:
            this.socket.send(payload);
            return [2 /*return*/];
        }
      });
    });
  };
  return TrajectoryPlugin;
})();
exports.default = TrajectoryPlugin;
