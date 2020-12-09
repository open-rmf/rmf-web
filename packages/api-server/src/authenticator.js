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
Object.defineProperty(exports, '__esModule', { value: true });
var fs = __importStar(require('fs'));
var jwt = __importStar(require('jsonwebtoken'));
function authenticator(options) {
  var secretOrPublicKey = (function () {
    if (options.type === 'publicKey') {
      return fs.readFileSync(options.secretOrPublicKey);
    } else {
      return options.secretOrPublicKey;
    }
  })();
  return function (socket, data, next) {
    if (socket.authorized) {
      next();
    }
    if (typeof data !== 'string') {
      throw new Error('expected token');
    }
    jwt.verify(data, secretOrPublicKey); // will throw if invalid
    socket.authorized = true;
    socket.send('ok');
    // don't call `next()`, authentication message should not be chained to downstream middlewares.
  };
}
exports.default = authenticator;
