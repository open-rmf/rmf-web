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
var __rest =
  (this && this.__rest) ||
  function (s, e) {
    var t = {};
    for (var p in s)
      if (Object.prototype.hasOwnProperty.call(s, p) && e.indexOf(p) < 0) t[p] = s[p];
    if (s != null && typeof Object.getOwnPropertySymbols === 'function')
      for (var i = 0, p = Object.getOwnPropertySymbols(s); i < p.length; i++) {
        if (e.indexOf(p[i]) < 0 && Object.prototype.propertyIsEnumerable.call(s, p[i]))
          t[p[i]] = s[p[i]];
      }
    return t;
  };
import React from 'react';
import { PrivateRouteBase } from '..';
export var PrivateRouteHOC = function (RouteComponent, RedirectComponent, hookUseLocation) {
  return function (_a) {
    var noRedirectToLogin = _a.noRedirectToLogin,
      children = _a.children,
      redirectPath = _a.redirectPath,
      unauthorized = _a.unauthorized,
      user = _a.user,
      rest = __rest(_a, ['noRedirectToLogin', 'children', 'redirectPath', 'unauthorized', 'user']);
    var location = hookUseLocation();
    var redirect = React.createElement(RedirectComponent, {
      to: { pathname: redirectPath, state: { from: location } },
    });
    return React.createElement(
      RouteComponent,
      __assign({}, rest),
      React.createElement(
        PrivateRouteBase,
        {
          user: user,
          noRedirectToLogin: noRedirectToLogin,
          unauthorized: unauthorized,
          redirect: redirect,
        },
        children,
      ),
    );
  };
};
export default PrivateRouteHOC;
