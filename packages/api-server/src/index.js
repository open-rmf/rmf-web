#!/usr/bin/env node
'use strict';
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
var fs = __importStar(require('fs'));
var http = __importStar(require('http'));
var https = __importStar(require('https'));
var ws_1 = __importDefault(require('ws'));
var yargs_1 = __importDefault(require('yargs'));
var api_gateway_1 = __importDefault(require('./api-gateway'));
var authenticator_1 = __importDefault(require('./authenticator'));
var logger_1 = __importDefault(require('./logger'));
var websocket_connect_1 = __importDefault(require('./websocket-connect'));
function loadPlugins() {
  return __awaiter(this, void 0, void 0, function () {
    var modules, plugins, _i, modules_1, module_1, plugin, e_1;
    return __generator(this, function (_a) {
      switch (_a.label) {
        case 0:
          modules = fs
            .readdirSync(__dirname + '/plugins', { withFileTypes: true })
            .filter(function (dirent) {
              return dirent.isDirectory();
            })
            .map(function (dirent) {
              return dirent.name;
            });
          plugins = {};
          (_i = 0), (modules_1 = modules);
          _a.label = 1;
        case 1:
          if (!(_i < modules_1.length)) return [3 /*break*/, 6];
          module_1 = modules_1[_i];
          _a.label = 2;
        case 2:
          _a.trys.push([2, 4, , 5]);
          return [
            4 /*yield*/,
            Promise.resolve().then(function () {
              return __importStar(require('./plugins/' + module_1));
            }),
          ];
        case 3:
          plugin = _a.sent();
          plugin.options(yargs_1.default);
          plugins[module_1] = plugin;
          return [3 /*break*/, 5];
        case 4:
          e_1 = _a.sent();
          logger_1.default.error('failed to load plugin "' + module_1 + '": ' + e_1);
          return [3 /*break*/, 5];
        case 5:
          _i++;
          return [3 /*break*/, 1];
        case 6:
          return [2 /*return*/, plugins];
      }
    });
  });
}
function main() {
  return __awaiter(this, void 0, void 0, function () {
    var yargs_, plugins, config, server, wsServer, app, api, _i, _a, _b, module_2, plugin, e_2;
    return __generator(this, function (_c) {
      switch (_c.label) {
        case 0:
          yargs_ = yargs_1.default
            .option('host', {
              description: 'host name or ip address to bind to',
              default: 'localhost',
              type: 'string',
            })
            .option('port', {
              default: 80,
              type: 'number',
            })
            .option('sslCert', {
              implies: 'sslKey',
              type: 'string',
            })
            .option('sslKey', {
              implies: 'sslCert',
              type: 'string',
            })
            .requiresArg(['sslCert', 'sslKey'])
            .option('secret', {
              description: 'secret used to verify auth token',
              type: 'string',
              conflicts: 'publicKey',
            })
            .option('publicKey', {
              description: 'pem encoded key used to verify auth token',
              type: 'string',
              conflicts: 'secret',
            })
            .config();
          return [4 /*yield*/, loadPlugins()];
        case 1:
          plugins = _c.sent();
          config = yargs_.argv;
          server = (function () {
            if (config.sslCert && config.sslKey) {
              var redirectServer = http.createServer();
              redirectServer.on('request', function (req, resp) {
                if (!req.url) {
                  resp.statusCode = 500;
                  resp.end();
                  return;
                }
                resp.statusCode = 301;
                var url = new URL(req.url);
                url.protocol = 'https';
                resp.setHeader('Location', url.href);
                resp.end();
              });
              return https.createServer({
                cert: fs.readFileSync(__dirname + '/' + config.sslCert),
                key: fs.readFileSync(__dirname + '/' + config.sslKey),
              });
            } else {
              logger_1.default.info('serving through http');
              return http.createServer();
            }
          })();
          wsServer = new ws_1.default.Server({ server: server });
          app = new websocket_connect_1.default(wsServer);
          if (config.secret) {
            app.use(authenticator_1.default({ type: 'secret', secretOrPublicKey: config.secret }));
          } else if (config.publicKey) {
            app.use(
              authenticator_1.default({ type: 'publicKey', secretOrPublicKey: config.publicKey }),
            );
          } else {
            logger_1.default.warn(
              'neither "secret" or "publicKey" is set, authentication is disabled',
            );
          }
          api = new api_gateway_1.default();
          (_i = 0), (_a = Object.entries(plugins));
          _c.label = 2;
        case 2:
          if (!(_i < _a.length)) return [3 /*break*/, 7];
          (_b = _a[_i]), (module_2 = _b[0]), (plugin = _b[1]);
          _c.label = 3;
        case 3:
          _c.trys.push([3, 5, , 6]);
          return [4 /*yield*/, plugin.onLoad(config, api)];
        case 4:
          _c.sent();
          logger_1.default.info('loaded plugin "' + module_2 + '"');
          return [3 /*break*/, 6];
        case 5:
          e_2 = _c.sent();
          logger_1.default.error('failed to load plugin "' + module_2 + '": ' + e_2);
          return [3 /*break*/, 6];
        case 6:
          _i++;
          return [3 /*break*/, 2];
        case 7:
          app.use(api.middleware);
          server.listen(config.port, config.host);
          return [2 /*return*/];
      }
    });
  });
}
main();
