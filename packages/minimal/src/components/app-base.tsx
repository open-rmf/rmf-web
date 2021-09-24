import { Grid, makeStyles } from '@material-ui/core';
import React from 'react';
import AppBar from './appbar';

const useStyles = makeStyles((theme) => ({
  appBase: {
    width: '100%',
    height: '100%',
    backgroundColor: theme.palette.background.default,
  },
}));

/**
 * Contains various components that are essential to the app and provides contexts to control them.
 * Components include:
 * - Settings
 * - Notifications
 *
 * Also provides `AppControllerContext` to allow children components to control them.
 */
export function AppBase({ children }: React.PropsWithChildren<{}>): JSX.Element | null {
  const classes = useStyles();

  return (
    <Grid container direction="column" className={classes.appBase} wrap="nowrap">
      <AppBar />
      {children}
    </Grid>
  );
}
