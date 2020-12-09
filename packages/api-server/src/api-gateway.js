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
var msgpack = __importStar(require('@msgpack/msgpack'));
var assert = __importStar(require('assert'));
var logger_1 = __importDefault(require('./logger'));
var ApiGateway = /** @class */ (function () {
  function ApiGateway() {
    var _this = this;
    this.middleware = function (socket, data, next) {
      _this._onMessage(socket, data, next);
    };
    this._rpcHandlers = {};
  }
  ApiGateway.prototype.registerHandler = function (method, cb) {
    this._rpcHandlers[method] = cb;
    logger_1.default.info('registered handler for "' + method + '"');
  };
  ApiGateway.prototype.getLogger = function (name) {
    return logger_1.default.child({ tag: name });
  };
  ApiGateway.prototype._onMessage = function (socket, data, next) {
    return __awaiter(this, void 0, void 0, function () {
      var req, buildResponse, sender, handler, handlerRet, e_1;
      return __generator(this, function (_a) {
        switch (_a.label) {
          case 0:
            assert.ok(data instanceof Buffer);
            req = msgpack.decode(data);
            assert.strictEqual('0', req.version);
            buildResponse = function (response) {
              return __assign({ version: '0', id: req.id }, response);
            };
            sender = {
              socket: socket,
              send: function (data) {
                return (
                  req.id !== undefined &&
                  req.id !== null &&
                  socket.send(msgpack.encode(buildResponse({ result: data, more: true })))
                );
              },
              end: function (data) {
                return (
                  req.id !== undefined &&
                  req.id !== null &&
                  socket.send(msgpack.encode(buildResponse({ result: data })))
                );
              },
              error: function (error) {
                return (
                  req.id !== undefined &&
                  req.id !== null &&
                  socket.send(msgpack.encode(buildResponse({ error: error })))
                );
              },
            };
            _a.label = 1;
          case 1:
            _a.trys.push([1, 3, , 4]);
            handler = this._rpcHandlers[req.method];
            if (!handler) {
              logger_1.default.warn('no handler for method "' + req.method + '"');
              sender.error({
                code: 1,
                message: 'no such method',
              });
              return [2 /*return*/];
            }
            return [4 /*yield*/, handler(req.params, sender)];
          case 2:
            handlerRet = _a.sent();
            /**
             * handler is "req -> resp" if it only has 1 argument, else it is a "stream" with messages
             * sent in chunks.
             *
             * "req -> resp" handler:
             *   * if it returns void (undefined), we send back "null" as the resp, because "result" is
             *     required on success.
             *   * if it returns any other value, send that as the result.
             * "stream" handler:
             *   * if it returns void (undefined), don't send anything back. The handler will take care
             *     of sending the result in chunks.
             *   * if it returns a value, send that as the result, but always set the "more" flag, the
             *     handler should take care of sending the rest of the chunks.
             */
            if (handlerRet === undefined) {
              if (this._rpcHandlers[req.method].length === 1) {
                sender.end(null);
              }
            } else {
              if (this._rpcHandlers[req.method].length === 1) {
                sender.end(handlerRet);
              } else {
                sender.send(handlerRet);
              }
            }
            return [3 /*break*/, 4];
          case 3:
            e_1 = _a.sent();
            sender.error({ code: 1, message: e_1.message });
            return [3 /*break*/, 4];
          case 4:
            next();
            return [2 /*return*/];
        }
      });
    });
  };
  return ApiGateway;
})();
exports.default = ApiGateway;
