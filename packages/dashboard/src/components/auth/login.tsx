import React from 'react';
import { Redirect } from 'react-router';
import { DASHBOARD_ROUTE } from '../../util/url';
import { AuthenticatorContext, UserContext } from '../auth/contexts';
import { getUrl, LoginBase } from 'rmf-auth';

export interface LoginProps {
  /**
   * defaults to url of `DASHBOARD_ROUTE`
   */
  successRedirectUri?: string;
}

export default function Login(props: LoginProps): JSX.Element {
  const successRedirectUri = props.successRedirectUri || getUrl(DASHBOARD_ROUTE);
  const authenticator = React.useContext(AuthenticatorContext);
  const user = React.useContext(UserContext);

  // Redirect is not working if use a component from a different package
  return user ? (
    <Redirect to={DASHBOARD_ROUTE} />
  ) : (
    <LoginBase
      authenticator={authenticator}
      title="Reporting"
      successRedirectUri={successRedirectUri}
    />
  );
}
