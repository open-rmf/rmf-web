import { Button, Typography, styled } from '@mui/material';
import React from 'react';

const prefix = 'login-card';
const classes = {
  container: `${prefix}-container`,
  title: `${prefix}-title`,
  logo: `${prefix}-logo`,
};
const StyledDiv = styled('div')(({ theme }) => ({
  [`&.${classes.container}`]: {
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
    borderStyle: 'none',
    borderRadius: 20,
    borderColor: 'black',
    padding: '70px',
    width: 'fit-content',
    minWidth: 250,
    backgroundColor: 'snow',
    boxShadow: theme.shadows[12],
  },
  [`& .${classes.title}`]: {
    color: '#44497a',
  },
  [`& .${classes.logo}`]: {
    width: 100,
    margin: '25px 0px 50px 0px',
  },
}));

export interface LoginCardProps extends React.PropsWithChildren<{}> {
  title: string;
  logo: string;
  onLoginClick?: React.MouseEventHandler;
}

export const LoginCard = React.forwardRef(
  (
    { title, logo, onLoginClick, children }: LoginCardProps,
    ref: React.Ref<HTMLDivElement>,
  ): JSX.Element => {
    return (
      <StyledDiv ref={ref} className={classes.container}>
        <Typography variant="h4" className={classes.title}>
          {title}
        </Typography>
        <img src={logo} alt="" className={classes.logo} />
        <Button variant="contained" aria-label="Login" onClick={onLoginClick}>
          Login
        </Button>
        {children}
      </StyledDiv>
    );
  },
);

export default LoginCard;
