import { Button } from '@material-ui/core';
import React from 'react';
import { Redirect } from 'react-router';
import { LoginPage } from '.';
import { Authenticator } from '../authenticator';
import { User } from '../user';

export interface LoginProps {
  /**
   * defaults to url of `DASHBOARD_ROUTE`
   */
  title: string;
  successRedirectUri: string;
  authenticator: Authenticator;
}

export const LoginBase = (props: LoginProps): JSX.Element => {
  const { successRedirectUri, authenticator, title } = props;

  async function handleRmfLogin(_event: React.MouseEvent): Promise<void> {
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
