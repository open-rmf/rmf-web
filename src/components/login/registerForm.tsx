import React, { useState, useEffect } from 'react';
import { TextField, Button, CircularProgress, Snackbar, Typography } from '@material-ui/core';
import authStyles from './auth-style';
import { Redirect } from 'react-router-dom';
import Alert from '@material-ui/lab/Alert';

const RegisterForm = function() {
  const classes = authStyles();

  const [username, setUsername] = useState('');
  const [firstName, setFirstName] = useState('');
  const [lastName, setLastName] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');

  const [showRegisterFail, setShowRegisterFail] = useState(false);
  const [errorMessage, setErrorMessage] = useState('');
  const [isButtonDisabled, setIsButtonDisabled] = useState(true);
  const [isSubmitting, setIsSubmitting] = useState(false);

  // TODO: remove this after implementing the authentication service
  const [redirect, setRedirect] = React.useState(false);

  useEffect(() => {
    if (
      username.trim() &&
      password.trim() &&
      firstName.trim() &&
      lastName.trim() &&
      confirmPassword.trim()
    ) {
      setIsButtonDisabled(false);
    } else {
      setIsButtonDisabled(true);
    }
  }, [username, password, firstName, lastName, confirmPassword]);

  function isFormValid() {
    let isValid = true;
    if (password !== confirmPassword) {
      setShowRegisterFail(true);
      setErrorMessage('Passwords do not match');
      isValid = false;
    }
    return isValid;
  }

  function handleRegisterFail(err: string): void {
    setIsSubmitting(false);
    setErrorMessage(err);
    setShowRegisterFail(true);
  }

  function handleRegisterSuccess() {
    setTimeout(() => {
      setIsSubmitting(false);
      setRedirect(true);
    }, 1000);
  }

  function submit(event: React.FormEvent<HTMLFormElement>): void {
    event.preventDefault();
    console.log(username, firstName, lastName, password, confirmPassword);

    if (isFormValid()) {
      // props.auth.register(username, firstName, lastName, password, confirmPassword);
      // TODO replace this logic with the a call to the authentication service
      if (username === 'user') {
        setIsSubmitting(true);
        handleRegisterSuccess();
      } else {
        // Replace with backend error message
        handleRegisterFail('Has been an error creating your account');
      }
    }
  }

  return (
    <div className={`${classes.flexColumnContainer} ${classes.fullPage}`}>
      {!!redirect && <Redirect to="/login" />}
      <div className={`${classes.flexColumnContainer} ${classes.authContainer}`}>
        <h1 className={classes.authTitle}>RoMi Dashboard</h1>
        <img src="assets/ros-health.png" alt="" className={classes.logo} />

        <form
          className={`${classes.authForm} ${classes.flexColumnContainer}`}
          onSubmit={submit}
          autoComplete="off"
        >
          <TextField
            id="username"
            label="Username"
            onChange={event => setUsername(event.target.value)}
            value={username}
            required
          />
          <TextField
            autoFocus
            id="fisrtName"
            label="First Name"
            onChange={event => setFirstName(event.target.value)}
            value={firstName}
            required
          />
          <TextField
            id="lastName"
            label="Last Name"
            onChange={event => setLastName(event.target.value)}
            value={lastName}
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
          <TextField
            id="confirmPassword"
            label="Confirm Password"
            onChange={event => setConfirmPassword(event.target.value)}
            required
            value={confirmPassword}
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
              REGISTER
            </Button>
          </div>
          {isSubmitting && <CircularProgress color="primary" />}
        </form>

        <Snackbar
          open={showRegisterFail}
          autoHideDuration={3000}
          onClose={() => setShowRegisterFail(false)}
        >
          <Alert onClose={() => setShowRegisterFail(false)} severity="error">
            {errorMessage}
          </Alert>
        </Snackbar>
      </div>
      <div className={classes.termOfServices}>
        <Typography>By creating an account you agree to the RoMi Terms of Service. </Typography>
      </div>
    </div>
  );
};

export default RegisterForm;
