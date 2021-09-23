import { styled } from '@material-ui/core';
import React from 'react';
import { LoginCard, LoginCardProps } from './login-card';

const classes = {
  container: 'login-page-container',
};
const LoginPageRoot = styled('div')(({ theme }) => ({
  [`&.${classes.container}`]: {
    width: '100vw',
    height: '100vh',
    position: 'absolute',
    left: 0,
    top: 0,
    backgroundColor: theme.palette.primary.main,
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
  },
}));

export const LoginPage = (props: LoginCardProps): JSX.Element => {
  return (
    <LoginPageRoot className={classes.container}>
      <LoginCard {...props} />
    </LoginPageRoot>
  );
};

export default LoginPage;
