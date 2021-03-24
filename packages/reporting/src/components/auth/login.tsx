import React from 'react';
import { Redirect } from 'react-router';
import { LoginBase } from 'rmf-auth';
import { DASHBOARD_ROUTE } from '../../util/url';
import { AuthenticatorContext, UserContext } from '../auth-contexts';

export interface LoginProps {
  /**
   * defaults to url of `DASHBOARD_ROUTE`
   */
  successRedirectUri?: string;
}

export default function Login(props: LoginProps): JSX.Element {
  const successRedirectUri = props.successRedirectUri || DASHBOARD_ROUTE;
  const authenticator = React.useContext(AuthenticatorContext);
  const user = React.useContext(UserContext);

  // Redirect is not working if use a component from a different package
  return user ? (
    <Redirect to={successRedirectUri} />
  ) : (
    <LoginBase
      authenticator={authenticator}
      title="Reporting"
      successRedirectUri={successRedirectUri}
    />
  );
}
