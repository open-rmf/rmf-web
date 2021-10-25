import { createMuiTheme, Grid, makeStyles, Snackbar, SnackbarProps } from '@material-ui/core';
import { Alert } from '@material-ui/lab';
import React from 'react';
import { rmfDark, ThemeProvider } from 'react-components';
import { loadSettings, saveSettings, Settings, ThemeMode } from '../settings';
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

const SnackbarAutoHideDuration = 2000;
const defaultTheme = createMuiTheme();

const useStyles = makeStyles({
  appBase: {
    width: '100%',
    height: '100%',
  },
});

export interface ManagedSnackbarProps extends Omit<SnackbarProps, 'open' | 'onClose'> {
  key: React.Key;
}

/**
 * Contains various components that are essential to the app and provides contexts to control them.
 * Components include:
 *
 * - Settings
 * - Help
 * - Tooltip
 * - Hotkeys reference
 * - Snackbars
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

  const theme = React.useMemo(() => {
    const preferDarkMode = settings.themeMode === ThemeMode.Dark;
    return preferDarkMode ? rmfDark : defaultTheme;
  }, [settings.themeMode]);

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

  /**
   * According to material design guidelines, multiple snackbars should display one after another
   * instead of stacking.
   */
  const [snackbars, setSnackbars] = React.useState<ManagedSnackbarProps[]>([]);
  const currentSnackbarProps: ManagedSnackbarProps | undefined = snackbars[0];
  const snackbarsCountersRef = React.useRef(0);

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
      showErrorAlert: (message, autoHideDuration = SnackbarAutoHideDuration) => {
        setSnackbars((prev) => [
          ...prev,
          {
            key: ++snackbarsCountersRef.current,
            autoHideDuration,
            children: <Alert severity="error">{message}</Alert>,
          },
        ]);
      },
      showSuccessAlert: (message, autoHideDuration = SnackbarAutoHideDuration) => {
        setSnackbars((prev) => [
          ...prev,
          {
            key: ++snackbarsCountersRef.current,
            autoHideDuration,
            children: <Alert severity="success">{message}</Alert>,
          },
        ]);
      },
    }),
    [updateSettings],
  );

  return (
    <ThemeProvider theme={theme}>
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
              <Snackbar
                open={!!currentSnackbarProps}
                onClose={() => setSnackbars((prev) => prev.slice(1))}
                {...currentSnackbarProps}
              ></Snackbar>
            </Grid>
          </AppControllerContext.Provider>
        </TooltipsContext.Provider>
      </SettingsContext.Provider>
    </ThemeProvider>
  );
}
