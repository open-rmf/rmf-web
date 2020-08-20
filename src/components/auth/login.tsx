import { Button, Typography } from '@material-ui/core';
import MuiAlert, { AlertProps } from '@material-ui/lab/Alert';
import React from 'react';
import { Redirect } from 'react-router';
import appConfig from '../../app-config';
import authStyles from './auth-style';

export function Alert(props: AlertProps): React.ReactElement {
  return <MuiAlert elevation={6} variant="filled" {...props} />;
}

export interface LoginProps {}

export default function Login(/* props: LoginProps */): React.ReactElement {
  const authenticator = appConfig.authenticator;
  const classes = authStyles();
  const [loginResponse, setLoginResponse] = React.useState(false);

  async function handleRmfLogin(_event: React.MouseEvent): Promise<void> {
    const redirectUri = new URL(window.location.href);
    redirectUri.searchParams.append('response', '1');
    authenticator.login();
  }

  React.useEffect(() => {
    (async () => {
      const query = new URLSearchParams(window.location.search);
      const resp = query.get('response');

      if (resp === '1') {
        setLoginResponse(true);
      }
    })();
  }, [authenticator]);

  return loginResponse ? (
    <Redirect to="/" />
  ) : (
    <div className={`${classes.flexColumnContainer} ${classes.fullPage}`}>
      <div className={`${classes.flexColumnContainer} ${classes.authContainer}`}>
        <Typography variant="h4" className={classes.authTitle}>
          RoMi Dashboard
        </Typography>
        <img src="assets/ros-health.png" alt="" className={classes.logo} />
        <Button id="login-button" onClick={handleRmfLogin} variant="contained">
          Login with RMF
        </Button>
      </div>
    </div>
  );
}
