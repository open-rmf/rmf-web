import Debug from 'debug';
import React from 'react';
var debug = Debug('PrivateRoute');
/**
 * This component validates if the user is authenticated before rendering component passed as a
 * prop.
 */
export var PrivateRouteBase = function (_a) {
  var noRedirectToLogin = _a.noRedirectToLogin,
    children = _a.children,
    unauthorized = _a.unauthorized,
    redirect = _a.redirect,
    user = _a.user;
  function render() {
    if (user) {
      return children;
    } else {
      if (!noRedirectToLogin) {
        debug('accessing private route while unauthenticated');
        debug('redirecting to login page');
        return redirect;
      } else {
        return unauthorized ? unauthorized : React.createElement('span', null, 'Unauthorized');
      }
    }
  }
  return React.createElement(React.Fragment, null, render());
};
export default PrivateRouteBase;
