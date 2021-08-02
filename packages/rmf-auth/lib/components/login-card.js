import { makeStyles, Typography } from '@material-ui/core';
import React from 'react';
var useStyles = makeStyles(function (theme) {
  return {
    container: {
      display: 'flex',
      flexDirection: 'column',
      alignItems: 'center',
      borderStyle: 'none',
      borderRadius: 20,
      borderColor: 'black',
      padding: '70px',
      width: 'fit-content',
      minWidth: 250,
      backgroundColor: 'snow',
      boxShadow: theme.shadows[12],
    },
    title: {
      color: '#44497a',
    },
    logo: {
      width: 100,
      margin: '25px 0px 50px 0px',
    },
  };
});
export var LoginCard = React.forwardRef(function (props, ref) {
  var title = props.title,
    logo = props.logo,
    children = props.children;
  var classes = useStyles();
  return React.createElement(
    'div',
    { ref: ref, className: classes.container },
    React.createElement(Typography, { variant: 'h4', className: classes.title }, title),
    React.createElement('img', { src: logo, alt: '', className: classes.logo }),
    children,
  );
});
export default LoginCard;
