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
import { makeStyles } from '@material-ui/core';
import React from 'react';
import { LoginCard } from './login-card';
var useStyles = makeStyles(function (theme) {
  return {
    container: {
      width: '100vw',
      height: '100vh',
      position: 'absolute',
      left: 0,
      top: 0,
      backgroundColor: theme.palette.primary.main,
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
    },
  };
});
export var LoginPage = function (props) {
  var classes = useStyles();
  return React.createElement(
    'div',
    { className: classes.container },
    React.createElement(LoginCard, __assign({}, props)),
  );
};
export default LoginPage;
