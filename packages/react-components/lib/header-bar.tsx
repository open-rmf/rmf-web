import { AppBar, AppBarProps, createStyles } from '@mui/material';
import { makeStyles } from '@mui/styles';
import clsx from 'clsx';
import React from 'react';

export type HeaderBarProps = React.PropsWithChildren<AppBarProps>;

const useStyles = makeStyles(() => ({
  root: {
    display: 'flex',
    flexDirection: 'row',
    alignItems: 'center',
    width: '100%',
  },
}));

export const HeaderBar = ({
  id = 'appbar',
  position = 'static',
  className,
  children,
}: HeaderBarProps): React.ReactElement => {
  const classes = useStyles();

  return (
    <AppBar id={id} position={position} className={clsx(classes.root, className)}>
      {children}
    </AppBar>
  );
};
