var __extends =
  (this && this.__extends) ||
  (function () {
    var extendStatics = function (d, b) {
      extendStatics =
        Object.setPrototypeOf ||
        ({ __proto__: [] } instanceof Array &&
          function (d, b) {
            d.__proto__ = b;
          }) ||
        function (d, b) {
          for (var p in b) if (Object.prototype.hasOwnProperty.call(b, p)) d[p] = b[p];
        };
      return extendStatics(d, b);
    };
    return function (d, b) {
      if (typeof b !== 'function' && b !== null)
        throw new TypeError('Class extends value ' + String(b) + ' is not a constructor or null');
      extendStatics(d, b);
      function __() {
        this.constructor = d;
      }
      d.prototype = b === null ? Object.create(b) : ((__.prototype = b.prototype), new __());
    };
  })();
import EventEmitter from 'eventemitter3';
var StubAuthenticator = /** @class */ (function (_super) {
  __extends(StubAuthenticator, _super);
  function StubAuthenticator() {
    var _this = (_super !== null && _super.apply(this, arguments)) || this;
    _this.user = 'stub';
    _this.token = undefined;
    return _this;
  }
  StubAuthenticator.prototype.init = function () {
    return Promise.resolve();
  };
  StubAuthenticator.prototype.login = function () {
    throw new Error('not supported');
  };
  StubAuthenticator.prototype.logout = function () {
    throw new Error('not supported');
  };
  StubAuthenticator.prototype.refreshToken = function () {
    return Promise.resolve();
  };
  return StubAuthenticator;
})(EventEmitter);
export default StubAuthenticator;
