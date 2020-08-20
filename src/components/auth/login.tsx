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
    authenticator.login();
  }

  React.useEffect(() => {
    (async () => {
      // if the current url is a starts with the oauth redirect url, we know that we came back
      // from an oauth response, in that case, redirect back to the default route.
      if (window.location.href.startsWith(appConfig.authRedirectUri)) {
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
