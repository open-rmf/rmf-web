import React from 'react';
import { createStyles, makeStyles, AppBar } from '@material-ui/core';

interface HeaderBarProps {
  children?: React.ReactNode;
  themeColor?: string;
}

const useStyles = makeStyles(() =>
  createStyles({
    root: {
      display: 'flex',
      flexDirection: 'row',
      width: '100%',
    },
  }),
);

export const HeaderBar = (props: HeaderBarProps): React.ReactElement => {
  const { children, themeColor } = props;
  const classes = useStyles();

  return (
    <AppBar id="appbar" position="relative" className={`${themeColor} ${classes.root}`}>
      {children}
    </AppBar>
  );
};
