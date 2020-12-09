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
var romi_js_rclnodejs_transport_1 = __importDefault(require('@osrf/romi-js-rclnodejs-transport'));
var fast_deep_equal_1 = __importDefault(require('fast-deep-equal'));
function options(yargs) {
  return yargs.option('ros2NodeName', {
    default: 'romi_dashboard_server',
    type: 'string',
  });
}
exports.options = options;
function onLoad(config, api) {
  return __awaiter(this, void 0, void 0, function () {
    var rosArgs, transport, plugin;
    return __generator(this, function (_a) {
      switch (_a.label) {
        case 0:
          rosArgs = process.env['RCLNODEJS_ROS_ARGS']
            ? JSON.parse(process.env['RCLNODEJS_ROS_ARGS'])
            : undefined;
          return [
            4 /*yield*/,
            romi_js_rclnodejs_transport_1.default.create(config.ros2NodeName, rosArgs),
          ];
        case 1:
          transport = _a.sent();
          plugin = new Ros2Plugin(transport, api.getLogger('ros2'));
          api.registerHandler('ros2Subscribe', function (params, send) {
            return plugin.subscribe(params, send);
          });
          api.registerHandler('ros2Unsubscribe', function (params) {
            return plugin.unsubscribe(params);
          });
          api.registerHandler('ros2CreatePublisher', function (params, send) {
            return plugin.createPublisher(params, send);
          });
          api.registerHandler('ros2Publish', function (params) {
            return plugin.publish(params);
          });
          api.registerHandler('ros2ServiceCall', function (params) {
            return plugin.serviceCall(params);
          });
          return [2 /*return*/];
      }
    });
  });
}
exports.onLoad = onLoad;
var Ros2Plugin = /** @class */ (function () {
  function Ros2Plugin(transport, _logger) {
    this.transport = transport;
    this._logger = _logger;
    this._subscriptions = {};
    this._innerSubscriptions = [];
    this._publishers = {};
    this._innerPublishers = [];
    this._idCounter = 0;
  }
  Object.defineProperty(Ros2Plugin.prototype, 'subscriptionCount', {
    get: function () {
      return Object.keys(this._subscriptions).length;
    },
    enumerable: false,
    configurable: true,
  });
  Object.defineProperty(Ros2Plugin.prototype, 'innerSubscriptionCount', {
    get: function () {
      return this._innerSubscriptions.length;
    },
    enumerable: false,
    configurable: true,
  });
  Object.defineProperty(Ros2Plugin.prototype, 'publisherCount', {
    get: function () {
      return Object.keys(this._publishers).length;
    },
    enumerable: false,
    configurable: true,
  });
  Object.defineProperty(Ros2Plugin.prototype, 'innerPublisherCount', {
    get: function () {
      return this._innerPublishers.length;
    },
    enumerable: false,
    configurable: true,
  });
  Ros2Plugin.prototype.subscribe = function (params, sender) {
    var _a;
    var _this = this;
    var id = this._idCounter++;
    sender.socket.once('close', function () {
      delete _this._subscriptions[id].callbacks[id];
      _this._logger.info('removed inner handler for subscription ' + id);
      delete _this._subscriptions[id];
      _this._logger.info('removed subscription ' + id);
    });
    var found = this._innerSubscriptions.find(function (record) {
      return fast_deep_equal_1.default(record.topic, params.topic);
    });
    var record;
    if (found) {
      record = found;
    } else {
      var newRecord_1 = {
        callbacks:
          ((_a = {}),
          (_a[id] = function (msg) {
            return sender.send({ message: msg });
          }),
          _a),
        subscription: this.transport.subscribe(this.toRomiTopic(params.topic), function (msg) {
          return Object.values(newRecord_1.callbacks).forEach(function (cb) {
            return cb(msg);
          });
        }),
        topic: params.topic,
      };
      this._logger.info('created inner subscription for "' + params.topic.topic + '"');
      this._innerSubscriptions.push(newRecord_1);
      record = newRecord_1;
    }
    record.callbacks[id] = function (msg) {
      return sender.send({ message: msg });
    };
    this._subscriptions[id] = record;
    return { id: id };
  };
  Ros2Plugin.prototype.unsubscribe = function (params) {
    var record = this._subscriptions[params.id];
    var id = params.id;
    if (record) {
      delete this._subscriptions[id].callbacks[id];
      this._logger.info('removed inner handler for subscription ' + id);
      delete this._subscriptions[id];
      this._logger.info('removed subscription ' + id);
    }
  };
  Ros2Plugin.prototype.createPublisher = function (params, sender) {
    var _this = this;
    var id = this._idCounter++;
    sender.socket.once('close', function () {
      delete _this._publishers[id];
      _this._logger.info('removed publisher ' + id);
    });
    var found = this._innerPublishers.find(function (record) {
      return fast_deep_equal_1.default(record.topic, params.topic);
    });
    var record;
    if (found) {
      record = found;
    } else {
      var newRecord = {
        publisher: this.transport.createPublisher(this.toRomiTopic(params.topic)),
        topic: params.topic,
      };
      this._logger.info('created inner publisher for "' + params.topic.topic + '"');
      record = newRecord;
      this._innerPublishers.push(record);
    }
    this._publishers[id] = record;
    return id;
  };
  Ros2Plugin.prototype.publish = function (params) {
    var record = this._publishers[params.id];
    record && record.publisher.publish(params.message);
  };
  Ros2Plugin.prototype.serviceCall = function (params) {
    return __awaiter(this, void 0, void 0, function () {
      return __generator(this, function (_a) {
        return [
          2 /*return*/,
          this.transport.call(this.toRomiService(params.service), params.request),
        ];
      });
    });
  };
  Ros2Plugin.prototype.toRomiTopic = function (ros2Topic) {
    return __assign(__assign({}, ros2Topic), {
      validate: function (msg) {
        return msg;
      },
    });
  };
  Ros2Plugin.prototype.toRomiService = function (ros2Service) {
    return __assign(__assign({}, ros2Service), {
      validateRequest: function (msg) {
        return msg;
      },
      validateResponse: function (msg) {
        return msg;
      },
    });
  };
  return Ros2Plugin;
})();
exports.default = Ros2Plugin;
