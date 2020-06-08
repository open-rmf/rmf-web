/**
 * WIP
 */
import { Snackbar, TextField, Typography, Button } from '@material-ui/core';
import React from 'react';
import { AuthService } from '../../auth-service';
import authStyles from './auth-style';
import { Redirect } from 'react-router-dom';

interface LoginProps {
  auth: AuthService;
}

export default function Login(props: LoginProps): JSX.Element {
  const classes = authStyles();
  const [showLoginFail, setShowLoginFail] = React.useState(false);
  const [user, setUser] = React.useState('');
  const [password, setPassword] = React.useState('');
  const [errorMessage, setErrorMessage] = React.useState('');
  // TODO: remove this after implementing the authentication service
  const [redirect, setRedirect] = React.useState(false);

  function handleLoginFail(err: string): void {
    setErrorMessage(err);
    setShowLoginFail(true);
    //   this._snackBar.open(err, undefined,
    //     { panelClass: 'login-failed-snackbar', duration: 3000 });
    //  this.user = '';
    //  this.password = '';
    //  this.inputUserView.nativeElement.focus();
  }

  function submit(event: React.FormEvent<HTMLFormElement>): void {
    event.preventDefault();
    console.log(user, password);
    // props.auth.login(user, password);
    handleLoginFail('asd');
    console.log('submit');
  }

  return (
    <div className={`${classes.flexColumnContainer} ${classes.fullPage}`}>
      {!!redirect && <Redirect to="/dashboard" />}
      <div className={`${classes.flexColumnContainer} ${classes.loginContainer}`}>
        <h1 className={classes.loginTitle}>RoMi Dashboard</h1>
        <img src="assets/ros-health.png" alt="" className={classes.logo} />
        <form className={`${classes.loginForm} ${classes.flexColumnContainer}`} onSubmit={submit}>
          <TextField label="User" onChange={event => setUser(event.target.value)} />
          <TextField label="Password" onChange={event => setPassword(event.target.value)} />

          <div className={classes.buttonContainer}>
            <Button
              variant="contained"
              color="primary"
              type="submit"
              className={classes.button}
              onClick={() => setRedirect(true)}
            >
              LOG IN
            </Button>
          </div>
          <div className={classes.resetPassword}>Need to reset your password?</div>
        </form>
      </div>
      <div className={classes.termOfServices}>
        By creating an account you agree to the RoMi Terms of Service.
      </div>
      <Snackbar
        open={showLoginFail}
        autoHideDuration={3000}
        onClose={() => setShowLoginFail(false)}
      >
        <Typography>{errorMessage}</Typography>
      </Snackbar>
    </div>
  );
}
