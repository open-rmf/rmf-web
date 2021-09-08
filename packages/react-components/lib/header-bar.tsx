import React from 'react';
import { createStyles, makeStyles, AppBar } from '@material-ui/core';

interface HeaderBarProps {
  children?: React.ReactNode;
  alarm: boolean;
}

const useStyles = makeStyles((theme) =>
  createStyles({
    root: {
      display: 'flex',
      flexDirection: 'row',
      alignItems: 'center',
      width: '100%',
    },
    alarmOn: {
      backgroundColor: theme.palette.error.main,
    },
    alarmOff: {
      backgroundColor: theme.palette.primary.main,
    },
  }),
);

export const HeaderBar = (props: HeaderBarProps): React.ReactElement => {
  const { children, alarm } = props;
  const classes = useStyles();

  const appbarClass = alarm
    ? `${classes.root} ${classes.alarmOn}`
    : `${classes.root} ${classes.alarmOff}`;

  return (
    <AppBar id="appbar" position="static" className={appbarClass}>
      {children}
    </AppBar>
  );
};
