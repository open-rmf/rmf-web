import { Grid, makeStyles, ThemeProvider } from '@material-ui/core';
import React from 'react';
import { rmfDark, rmfLight, GlobalCss } from 'react-components';
import { loadSettings, saveSettings, ThemeMode } from '../settings';
import { ErrorSnackbar } from 'react-components';
import {
  AppController,
  AppControllerContext,
  SettingsContext,
  Tooltips,
  TooltipsContext,
} from './app-contexts';
import { AppDrawers } from './app-drawers';
import AppBar from './appbar';

const useStyles = makeStyles((theme) => ({
  appBase: {
    width: '100%',
    height: '100%',
  },
}));

/**
 * Contains various components that are essential to the app and provides contexts to control them.
 * Components include:
 *
 * - Settings
 * - Help
 * - Tooltip
 * - Hotkeys reference
 * - Notifications
 *
 * Also provides `AppControllerContext` to allow children components to control them.
 */
export function AppBase({ children }: React.PropsWithChildren<{}>): JSX.Element | null {
  const classes = useStyles();

  const [settings, setSettings] = React.useState(() => loadSettings());
  const [showSettings, setShowSettings] = React.useState(false);
  const [showHelp, setShowHelp] = React.useState(false);
  const [showHotkeysDialog, setShowHotkeysDialog] = React.useState(false);
  const [showTooltips, setShowTooltips] = React.useState(false);
  const [showErrorAlert, setShowErrorAlert] = React.useState(false);
  const [errorMessage, setErrorMessage] = React.useState('');

  const theme = React.useMemo(() => {
    const preferDarkMode = settings.themeMode === ThemeMode.Dark;
    return preferDarkMode ? rmfDark : rmfLight;
  }, [settings.themeMode]);

  const tooltips = React.useMemo<Tooltips>(
    () => ({
      showTooltips,
    }),
    [showTooltips],
  );

  const appController = React.useMemo<AppController>(
    () => ({
      showSettings: setShowSettings,
      setSettings,
      saveSettings,
      toggleSettings: () => setShowSettings((prev) => !prev),
      showHelp: setShowHelp,
      toggleHelp: () => setShowHelp((prev) => !prev),
      showHotkeysDialog: setShowHotkeysDialog,
      toggleHotkeysDialog: () => setShowHotkeysDialog((prev) => !prev),
      showTooltips: setShowTooltips,
      toggleTooltips: () => setShowTooltips((prev) => !prev),
      showErrorAlert: (message) => {
        setErrorMessage(message);
        setShowErrorAlert(true);
      },
    }),
    [],
  );

  return (
    <ThemeProvider theme={theme}>
      <GlobalCss />
      <SettingsContext.Provider value={settings}>
        <TooltipsContext.Provider value={tooltips}>
          <AppControllerContext.Provider value={appController}>
            <Grid container direction="column" className={classes.appBase} wrap="nowrap">
              <AppBar />
              {children}
              <AppDrawers
                settings={settings}
                showHelp={showHelp}
                showHotkeysDialog={showHotkeysDialog}
                showSettings={showSettings}
              />
              <ErrorSnackbar
                open={showErrorAlert}
                message={errorMessage}
                onClose={() => setShowErrorAlert(false)}
              />
            </Grid>
          </AppControllerContext.Provider>
        </TooltipsContext.Provider>
      </SettingsContext.Provider>
    </ThemeProvider>
  );
}
