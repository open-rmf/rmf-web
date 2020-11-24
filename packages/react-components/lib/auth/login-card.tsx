import { makeStyles, Typography } from '@material-ui/core';
import React from 'react';

const useStyles = makeStyles((theme) => ({
  container: {
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
  title: {
    color: '#44497a',
  },
  logo: {
    width: 100,
    margin: '25px 0px 50px 0px',
  },
}));

export interface LoginCardProps {
  title: string;
  logo: string;
  children?: React.ReactNode;
}

export const LoginCard = React.forwardRef(
  (props: LoginCardProps, ref: React.Ref<HTMLDivElement>): JSX.Element => {
    const { title, logo, children } = props;
    const classes = useStyles();

    return (
      <div ref={ref} className={classes.container}>
        <Typography variant="h4" className={classes.title}>
          {title}
        </Typography>
        <img src={logo} alt="" className={classes.logo} />
        {children}
      </div>
    );
  },
);

export default LoginCard;
