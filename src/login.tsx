/**
 * WIP
 */
import { makeStyles, Snackbar, TextField } from '@material-ui/core';
import { Alert } from '@material-ui/lab';
import React from 'react';
import { AuthService } from './auth-service';

const useStyles = makeStyles(theme => ({
  flexColumnContainer: {
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
  },
  fullPage: {
    width: '100%',
    height: '100%',
    backgroundColor: '#304060',
  },
  loginContainer: {
    borderStyle: 'none',
    borderRadius: 20,
    borderColor: 'black',
    padding: 50,
    width: 'fit-content',
    minWidth: 250,
    margin: 'auto',
    backgroundColor: 'snow',
    boxShadow: theme.shadows[12],
  },
  loginTitle: {
    color: '#304060',
    marginBottom: 50,
  },
  loginForm: {
    marginTop: 50,
  },
  loginFailSnackbar: {
    marginBottom: 50,
  },
}));

interface LoginProps {
  auth: AuthService;
}

export default function Login(props: LoginProps): JSX.Element {
  const classes = useStyles();
  const [showLoginFail, setShowLoginFail] = React.useState(false);
  const [user, setUser] = React.useState('');
  const [password, setPassword] = React.useState('');
  const [errorMessage, setErrorMessage] = React.useState('');

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
      <div className={`${classes.flexColumnContainer} ${classes.loginContainer}`}>
        <h1 className={classes.loginTitle}>RoMi Dashboard</h1>
        <img src="assets/ros-health.png" alt="" />
        <form className={`${classes.loginForm} ${classes.flexColumnContainer}`} onSubmit={submit}>
          <TextField label="User" onChange={event => setUser(event.target.value)} />
          <TextField label="Password" onChange={event => setPassword(event.target.value)} />
          <button type="submit" style={{ display: 'none' }} />
        </form>
      </div>
      <Snackbar
        open={showLoginFail}
        autoHideDuration={3000}
        onClose={() => setShowLoginFail(false)}
      >
        <Alert className={classes.loginFailSnackbar} severity="error">
          {errorMessage}
        </Alert>
      </Snackbar>
    </div>
  );
}
