import { Button, Typography } from '@material-ui/core';
import MuiAlert, { AlertProps } from '@material-ui/lab/Alert';
import React from 'react';
import authStyles from './auth-style';
import AuthContext from './context';
import { Redirect } from 'react-router';

export function Alert(props: AlertProps): React.ReactElement {
  return <MuiAlert elevation={6} variant="filled" {...props} />;
}

export default function Login(_props: {}): React.ReactElement {
  const classes = authStyles();
  const auth = React.useContext(AuthContext);
  const [loginSuccess, setLoginSuccess] = React.useState(false);

  async function handleRmfLogin(_event: React.MouseEvent): Promise<void> {
    const redirectUri = new URL(window.location.href);
    redirectUri.searchParams.append('success', '1');
    auth?.login(redirectUri.href);
  }

  React.useEffect(() => {
    (async () => {
      const query = new URLSearchParams(window.location.search);
      const success = query.get('success');

      if (success === '1') {
        setLoginSuccess(true);
      }
    })();
  }, [auth]);

  return loginSuccess ? (
    <Redirect to="/" />
  ) : (
    <div className={`${classes.flexColumnContainer} ${classes.fullPage}`}>
      <div className={`${classes.flexColumnContainer} ${classes.authContainer}`}>
        <Typography variant="h4" className={classes.authTitle}>
          RoMi Dashboard
        </Typography>
        <img src="assets/ros-health.png" alt="" className={classes.logo} />
        <Button onClick={handleRmfLogin} variant="contained">
          Login with RMF
        </Button>
      </div>
    </div>
  );
}
