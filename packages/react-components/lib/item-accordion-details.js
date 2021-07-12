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
import { makeStyles } from '@material-ui/core';
import AccordionDetails from '@material-ui/core/AccordionDetails';
import React from 'react';
import { joinClasses } from './css-utils';
var useStyles = makeStyles({
  details: {
    flexFlow: 'column',
    padding: 0,
  },
});
export var ItemAccordionDetails = function (props) {
  var className = props.className,
    otherProps = __rest(props, ['className']);
  var classes = useStyles();
  return React.createElement(
    AccordionDetails,
    __assign({ className: joinClasses(classes.details, className) }, otherProps),
  );
};
export default ItemAccordionDetails;
