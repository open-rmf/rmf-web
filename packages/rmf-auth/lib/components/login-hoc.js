import React from 'react';
import { LoginBase } from '..';
// Redirect is not working if use a component from a different package, so we are using a HOC to wrap the Redirect
export var LoginHOC = function (RedirectComponent) {
  return function LoginTest(props) {
    var user = props.user,
      authenticator = props.authenticator,
      successRedirectUri = props.successRedirectUri,
      title = props.title;
    return user
      ? React.createElement(RedirectComponent, { to: successRedirectUri })
      : React.createElement(LoginBase, {
          authenticator: authenticator,
          title: title,
          successRedirectUri: successRedirectUri,
        });
  };
};
