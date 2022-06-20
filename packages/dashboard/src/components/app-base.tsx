import { Alert, AlertProps, createTheme, Grid, Snackbar } from '@mui/material';
import { ThemeProvider } from '@mui/material/styles';
import React from 'react';
import { rmfDark } from 'react-components';
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

const DefaultAlertDuration = 2000;
const defaultTheme = createTheme();

export interface AppBaseProps {
  extraToolbarItems?: React.ReactNode;
}

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
export function AppBase({
  extraToolbarItems,
  children,
}: React.PropsWithChildren<AppBaseProps>): JSX.Element | null {
  const [settings, setSettings] = React.useState(() => loadSettings());
  const [showSettings, setShowSettings] = React.useState(false);
  const [showHelp, setShowHelp] = React.useState(false);
  const [showHotkeysDialog, setShowHotkeysDialog] = React.useState(false);
  const [showTooltips, setShowTooltips] = React.useState(false);
  const [showAlert, setShowAlert] = React.useState(false);
  const [alertSeverity, setAlertSeverity] = React.useState<AlertProps['severity']>('error');
  const [alertMessage, setAlertMessage] = React.useState('');
  const [alertDuration, setAlertDuration] = React.useState(DefaultAlertDuration);
  const [extraAppbarIcons, setExtraAppbarIcons] = React.useState<React.ReactNode>(null);

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
      showAlert: (severity, message, autoHideDuration) => {
        setAlertSeverity(severity);
        setAlertMessage(message);
        setShowAlert(true);
        setAlertDuration(autoHideDuration || DefaultAlertDuration);
      },
      setExtraAppbarIcons,
    }),
    [updateSettings],
  );

  return (
    <ThemeProvider theme={theme}>
      <SettingsContext.Provider value={settings}>
        <TooltipsContext.Provider value={tooltips}>
          <AppControllerContext.Provider value={appController}>
            <Grid
              container
              direction="column"
              style={{ width: '100%', height: '100%' }}
              wrap="nowrap"
            >
              <AppBar extraToolbarItems={extraAppbarIcons} />
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
              {/* TODO: Support stacking of alerts */}
              <Snackbar
                open={showAlert}
                message={alertMessage}
                onClose={() => setShowAlert(false)}
                autoHideDuration={alertDuration}
              >
                <Alert
                  onClose={() => setShowAlert(false)}
                  severity={alertSeverity}
                  sx={{ width: '100%' }}
                >
                  {alertMessage}
                </Alert>
              </Snackbar>
            </Grid>
          </AppControllerContext.Provider>
        </TooltipsContext.Provider>
      </SettingsContext.Provider>
    </ThemeProvider>
  );
}
