import { Grid, makeStyles } from '@material-ui/core';
import React from 'react';
import { ErrorSnackbar } from 'react-components';
import { loadSettings, saveSettings, Settings } from '../settings';
import {
  AppController,
  AppControllerContext,
  SettingsContext,
  Tooltips,
  TooltipsContext,
} from './app-contexts';
import AppBar from './appbar';
import HelpDrawer from './drawers/help-drawer';
import HotKeysDialog from './drawers/hotkeys-dialog';
import SettingsDrawer from './drawers/settings-drawer';

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

  const tooltips = React.useMemo<Tooltips>(
    () => ({
      showTooltips,
    }),
    [showTooltips],
  );

  const updateSettings = React.useCallback((newSettings: Settings) => {
    saveSettings(newSettings);
    setSettings(newSettings);
  }, []);

  const appController = React.useMemo<AppController>(
    () => ({
      setShowSettings,
      updateSettings,
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
    [updateSettings],
  );

  return (
    <SettingsContext.Provider value={settings}>
      <TooltipsContext.Provider value={tooltips}>
        <AppControllerContext.Provider value={appController}>
          <Grid container direction="column" className={classes.appBase} wrap="nowrap">
            <AppBar />
            {children}
            <SettingsDrawer
              open={showSettings}
              settings={settings}
              onSettingsChange={(settings) => {
                setSettings(settings);
                saveSettings(settings);
              }}
              handleCloseButton={() => setShowSettings(false)}
            />
            <HelpDrawer
              open={showHelp}
              handleCloseButton={() => setShowHelp(false)}
              onClose={() => setShowHelp(false)}
              setShowHotkeyDialog={() => setShowHotkeysDialog(true)}
              showTour={() => {
                setShowHelp(false);
              }}
            />

            {showHotkeysDialog && (
              <HotKeysDialog
                open={showHotkeysDialog}
                handleClose={() => setShowHotkeysDialog(false)}
              />
            )}
            <ErrorSnackbar
              open={showErrorAlert}
              message={errorMessage}
              onClose={() => setShowErrorAlert(false)}
            />
          </Grid>
        </AppControllerContext.Provider>
      </TooltipsContext.Provider>
    </SettingsContext.Provider>
  );
}
