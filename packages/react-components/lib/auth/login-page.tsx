import { makeStyles } from '@material-ui/core';
import React from 'react';
import { LoginCard, LoginCardProps } from './login-card';

const useStyles = makeStyles({
  container: {
    width: '100vw',
    height: '100vh',
    position: 'absolute',
    left: 0,
    top: 0,
    backgroundColor: '#44497a',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
  },
});

export const LoginPage = (props: LoginCardProps): JSX.Element => {
  const classes = useStyles();
  return (
    <div className={classes.container}>
      <LoginCard {...props} />
    </div>
  );
};

export default LoginPage;
