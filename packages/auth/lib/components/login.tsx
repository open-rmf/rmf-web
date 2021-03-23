import { Button } from '@material-ui/core';
import React from 'react';
import { Redirect } from 'react-router';
import { LoginPage } from '.';
import Authenticator from '../authenticator';
import { User } from '../user';

export interface LoginProps {
  /**
   * defaults to url of `DASHBOARD_ROUTE`
   */
  successRedirectUri?: string;
  defaultRoute: string;
  user?: User;
  authenticator: Authenticator;
}

export const Login = (props: LoginProps): JSX.Element => {
  const { defaultRoute, authenticator, user } = props;
  const successRedirectUri = props.successRedirectUri || props.defaultRoute; //getUrl(DASHBOARD_ROUTE);
  // const authenticator = React.useContext(AuthenticatorContext);
  // const user = React.useContext(UserContext);

  async function handleRmfLogin(_event: React.MouseEvent): Promise<void> {
    authenticator.login(successRedirectUri);
  }

  return user ? (
    <Redirect to={successRedirectUri} />
  ) : (
    <LoginPage title="Dashboard" logo="assets/ros-health.png">
      <Button id="login-button" onClick={handleRmfLogin} variant="contained">
        Login
      </Button>
    </LoginPage>
  );
};

export default Login;
