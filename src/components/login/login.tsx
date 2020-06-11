/**
 * WIP
 */
import { Snackbar, TextField, Button, CircularProgress } from '@material-ui/core';
import React, { useState, useEffect } from 'react';
import { AuthService } from '../../auth-service';
import authStyles from './auth-style';
import { Redirect } from 'react-router-dom';
import MuiAlert, { AlertProps } from '@material-ui/lab/Alert';
import { Typography } from '@material-ui/core';
import { DASHBOARD_ROUTE } from '../../util/url';

interface LoginProps {
  auth: AuthService;
}

export function Alert(props: AlertProps) {
  return <MuiAlert elevation={6} variant="filled" {...props} />;
}

export default function Login(props: LoginProps): JSX.Element {
  const classes = authStyles();
  const [showLoginFail, setShowLoginFail] = useState(false);
  const [user, setUser] = useState('');
  const [password, setPassword] = useState('');
  const [errorMessage, setErrorMessage] = useState('');
  const [isButtonDisabled, setIsButtonDisabled] = useState(true);
  const [isSubmitting, setIsSubmitting] = useState(false);
  // TODO: remove this after implementing the authentication service
  const [redirect, setRedirect] = React.useState(false);

  useEffect(() => {
    if (user.trim() && password.trim()) {
      setIsButtonDisabled(false);
    } else {
      setIsButtonDisabled(true);
    }
  }, [user, password]);

  function cleanForm() {
    setUser('');
    setPassword('');
  }

  function handleLoginFail(err: string): void {
    setIsSubmitting(false);
    setErrorMessage(err);
    setShowLoginFail(true);
    cleanForm();
  }

  function handleLoginSuccess() {
    setTimeout(() => {
      setIsSubmitting(false);
      setRedirect(true);
    }, 1000);
  }

  function submit(event: React.FormEvent<HTMLFormElement>): void {
    event.preventDefault();
    console.log(user, password);
    setIsSubmitting(true);
    // props.auth.login(user, password);
    // TODO replace this logic with the a call to the authentication service
    if (user === 'user' && password === 'password') {
      handleLoginSuccess();
    } else {
      handleLoginFail('Incorrect username or password');
    }
  }

  return (
    <div className={`${classes.flexColumnContainer} ${classes.fullPage}`}>
      {!!redirect && <Redirect to={DASHBOARD_ROUTE} />}
      <div className={`${classes.flexColumnContainer} ${classes.authContainer}`}>
        <h1 className={classes.authTitle}>RoMi Dashboard</h1>
        <img src="assets/ros-health.png" alt="" className={classes.logo} />

        <form
          className={`${classes.authForm} ${classes.flexColumnContainer}`}
          onSubmit={submit}
          autoComplete="off"
        >
          <TextField
            autoFocus
            id="username"
            label="Username"
            onChange={event => setUser(event.target.value)}
            value={user}
            required
          />
          <TextField
            id="password"
            label="Password"
            onChange={event => setPassword(event.target.value)}
            required
            value={password}
            type="password"
          />

          <div className={classes.buttonContainer}>
            <Button
              className={classes.button}
              color="primary"
              disabled={isButtonDisabled}
              type="submit"
              variant="contained"
            >
              LOG IN
            </Button>
          </div>
          {isSubmitting && <CircularProgress color="primary" />}
          <div className={classes.resetPassword}>
            <Typography>Need to reset your password?</Typography>
          </div>
        </form>

        <Snackbar
          open={showLoginFail}
          autoHideDuration={3000}
          onClose={() => setShowLoginFail(false)}
        >
          <Alert onClose={() => setShowLoginFail(false)} severity="error">
            {errorMessage}
          </Alert>
        </Snackbar>
      </div>
      <div className={classes.termOfServices}>
        <Typography>By creating an account you agree to the RoMi Terms of Service.</Typography>
      </div>
    </div>
  );
}
