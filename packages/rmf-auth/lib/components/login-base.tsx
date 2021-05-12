import { Button } from '@material-ui/core';
import React from 'react';
import { LoginPage } from '.';
import { Authenticator } from '../authenticator';

export interface LoginBaseProps {
  /**
   * defaults to url of `DASHBOARD_ROUTE`
   */
  title: string;
  successRedirectUri: string;
  authenticator: Authenticator;
}

export const LoginBase = (props: LoginBaseProps): JSX.Element => {
  const { successRedirectUri, authenticator, title } = props;

  async function handleRmfLogin(): Promise<void> {
    authenticator.login(successRedirectUri);
  }

  return (
    <LoginPage title={title} logo="assets/ros-health.png">
      <Button id="login-button" onClick={handleRmfLogin} variant="contained">
        Login
      </Button>
    </LoginPage>
  );
};

export default LoginBase;
