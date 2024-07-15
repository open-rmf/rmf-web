import {
  Alert,
  AlertProps,
  Backdrop,
  CircularProgress,
  createTheme,
  CssBaseline,
  Grid,
  Snackbar,
} from '@mui/material';
import { ThemeProvider } from '@mui/material/styles';
import React from 'react';
import { rmfDark, rmfLight } from 'react-components';
import { loadSettings, saveSettings, Settings, ThemeMode } from '../settings';
import { AppController, AppControllerContext, SettingsContext } from './app-contexts';
import AppBar from './appbar';
import { AlertStore } from './alert-store';
import { AppEvents } from './app-events';
import { DeliveryAlertStore } from './delivery-alert-store';

const DefaultAlertDuration = 2000;
const defaultTheme = createTheme({
  typography: {
    fontSize: 16,
  },
});

/**
 * Contains various components that are essential to the app and provides contexts to control them.
 * Components include:
 *
 * - Settings
 * - Alerts
 *
 * Also provides `AppControllerContext` to allow children components to control them.
 */
export function AppBase({ children }: React.PropsWithChildren<{}>): JSX.Element | null {
  const [settings, setSettings] = React.useState(() => loadSettings());
  const [showAlert, setShowAlert] = React.useState(false);
  const [alertSeverity, setAlertSeverity] = React.useState<AlertProps['severity']>('error');
  const [alertMessage, setAlertMessage] = React.useState('');
  const [alertDuration, setAlertDuration] = React.useState(DefaultAlertDuration);
  const [extraAppbarIcons, setExtraAppbarIcons] = React.useState<React.ReactNode>(null);
  const [openLoadingBackdrop, setOpenLoadingBackdrop] = React.useState(false);

  const theme = React.useMemo(() => {
    switch (settings.themeMode) {
      case ThemeMode.RmfLight:
        return rmfLight;
      case ThemeMode.RmfDark:
        return rmfDark;
      default:
        return defaultTheme;
    }
  }, [settings.themeMode]);

  const updateSettings = React.useCallback((newSettings: Settings) => {
    saveSettings(newSettings);
    setSettings(newSettings);
  }, []);

  const appController = React.useMemo<AppController>(
    () => ({
      updateSettings,
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

  React.useEffect(() => {
    const sub = AppEvents.loadingBackdrop.subscribe((value) => {
      setOpenLoadingBackdrop(value);
    });
    return () => sub.unsubscribe();
  }, []);

  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />
      {openLoadingBackdrop && (
        <Backdrop
          sx={{ color: '#fff', zIndex: (theme) => theme.zIndex.drawer + 1 }}
          open={openLoadingBackdrop}
        >
          <CircularProgress color="inherit" />
        </Backdrop>
      )}
      <SettingsContext.Provider value={settings}>
        <AppControllerContext.Provider value={appController}>
          <AlertStore />
          <DeliveryAlertStore />
          <Grid
            container
            direction="column"
            style={{ width: '100%', height: '100%' }}
            wrap="nowrap"
          >
            <AppBar extraToolbarItems={extraAppbarIcons} />
            {children}
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
      </SettingsContext.Provider>
    </ThemeProvider>
  );
}
