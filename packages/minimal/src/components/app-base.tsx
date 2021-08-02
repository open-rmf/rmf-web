import { Grid, makeStyles } from '@material-ui/core';
import React from 'react';
import { loadSettings, saveSettings } from '../settings';
import { AppController, AppControllerContext, SettingsContext } from './app-contexts';
import { AppDrawers } from './app-drawers';
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

  const [settings, setSettings] = React.useState(() => loadSettings());
  const [showSettings, setShowSettings] = React.useState(false);

  const appController = React.useMemo<AppController>(
    () => ({
      showSettings: setShowSettings,
      setSettings,
      saveSettings,
      toggleSettings: () => setShowSettings((prev) => !prev),
    }),
    [],
  );

  return (
    <SettingsContext.Provider value={settings}>
      <AppControllerContext.Provider value={appController}>
        <Grid container direction="column" className={classes.appBase} wrap="nowrap">
          <AppBar />
          {children}
          <AppDrawers settings={settings} showSettings={showSettings} />
        </Grid>
      </AppControllerContext.Provider>
    </SettingsContext.Provider>
  );
}
