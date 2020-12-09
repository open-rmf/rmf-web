'use strict';
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
var logger_1 = __importDefault(require('./logger'));
var WebSocketConnect = /** @class */ (function () {
  function WebSocketConnect(server) {
    var _this = this;
    this.server = server;
    this.stack = [];
    server.on('connection', function (socket, req) {
      var logger = logger_1.default.child({ tag: req.socket.remoteAddress });
      logger.info('connection established');
      socket.on('message', function (data) {
        return __awaiter(_this, void 0, void 0, function () {
          var _loop_1, _i, _a, mdw, e_1;
          return __generator(this, function (_b) {
            switch (_b.label) {
              case 0:
                _b.trys.push([0, 5, , 6]);
                _loop_1 = function (mdw) {
                  return __generator(this, function (_a) {
                    switch (_a.label) {
                      case 0:
                        return [
                          4 /*yield*/,
                          new Promise(function (res) {
                            return mdw(socket, data, res);
                          }),
                        ];
                      case 1:
                        _a.sent();
                        return [2 /*return*/];
                    }
                  });
                };
                (_i = 0), (_a = this.stack);
                _b.label = 1;
              case 1:
                if (!(_i < _a.length)) return [3 /*break*/, 4];
                mdw = _a[_i];
                return [5 /*yield**/, _loop_1(mdw)];
              case 2:
                _b.sent();
                _b.label = 3;
              case 3:
                _i++;
                return [3 /*break*/, 1];
              case 4:
                return [3 /*break*/, 6];
              case 5:
                e_1 = _b.sent();
                logger.info('[' + req.socket.remoteAddress + '] ' + e_1);
                socket.close();
                logger.info('connection closed');
                return [3 /*break*/, 6];
              case 6:
                return [2 /*return*/];
            }
          });
        });
      });
    });
  }
  WebSocketConnect.prototype.use = function (mdw) {
    this.stack.push(mdw);
  };
  return WebSocketConnect;
})();
exports.default = WebSocketConnect;
