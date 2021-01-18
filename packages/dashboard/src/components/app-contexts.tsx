import { createMuiTheme, ThemeProvider } from '@material-ui/core';
import { Settings } from 'http2';
import React from 'react';
import appConfig from '../app-config';
import ResourceManager from '../resource-manager';
import { defaultSettings, loadSettings } from '../settings';
import { UserContext } from './auth/contexts';
import { User } from './auth/user';
import { DashboardDrawers } from './dashboard-drawers';
import LoadingScreen, { LoadingScreenProps } from './loading-screen';
import NotificationBar, { NotificationBarProps } from './notification-bar';

/* Declares the ResourcesContext which contains the resources used on the app*/
export const ResourcesContext = React.createContext<ResourceManager | undefined>(undefined);

export const SettingsContext = React.createContext(defaultSettings());

export interface AppController {
  showSettings(show: boolean): void;
  saveSettings(settings: Settings): void;
  showHelp(show: boolean): void;
  showHotkeysDialog(show: boolean): void;
}

export const AppControllerContext = React.createContext<AppController>({
  showSettings: () => {},
  saveSettings: () => {},
  showHelp: () => {},
  showHotkeysDialog: () => {},
});

export const NotificationBarContext = React.createContext<React.Dispatch<
  React.SetStateAction<NotificationBarProps | null>
> | null>(null);

export const TooltipContext = React.createContext({
  showTooltips: true,
  toggleTooltips: () => {},
});

export const LoadingScreenContext = React.createContext<React.Dispatch<LoadingScreenProps>>(
  () => {},
);

export interface AppContextProviderProps extends React.PropsWithChildren<{}> {}

const theme = createMuiTheme({
  palette: {
    primary: {
      main: '#44497a',
      dark: '#323558',
      light: '#565d99',
    },
  },
});

/**
 * Contains various components that are essential to the app and provides contexts to control them.
 * Components include:
 *
 * - Settings
 * - Help
 * - Tooltip
 * - Hotkeys reference
 * - Notifications
 * - Snackbars
 *
 * Also provides `AppControllerContext` to allow children components to control them.
 */
export function AppBase(props: AppContextProviderProps): JSX.Element | null {
  const { children } = props;

  const [authInitialized, setAuthInitialized] = React.useState(false);
  const [user, setUser] = React.useState<User | null>(null);

  const [settings, setSettings] = React.useState(() => loadSettings());
  const [showSettings, setShowSettings] = React.useState(false);

  const authenticator = appConfig.authenticator;

  const resourceManager = React.useRef<ResourceManager | undefined>(undefined);

  const [loadingScreenProps, setLoadingScreenProps] = React.useState<LoadingScreenProps>({});

  const [
    notificationBarMessage,
    setNotificationBarMessage,
  ] = React.useState<NotificationBarProps | null>(null);

  const [showTooltips, setShowTooltips] = React.useState(true);
  const toggleTooltips = React.useCallback(() => {
    setShowTooltips(!showTooltips);
  }, [showTooltips]);

  const [appReady, setAppReady] = React.useState(false);

  /**
   * If resource loading gets too long we should add a loading screen.
   */
  React.useEffect(() => {
    if (!appConfig.appResources) {
      setAppReady(true);
      return;
    }
    (async () => {
      resourceManager.current = await appConfig.appResources;
      setAppReady(true);
    })();
  }, []);

  React.useEffect(() => {
    (async () => {
      authenticator.on('userChanged', (newUser) => setUser(newUser));
      await authenticator.init();
      setUser(authenticator.user || null);
      setAuthInitialized(true);
    })();
  }, [authenticator]);

  return authInitialized && appReady ? (
    <SettingsContext.Provider value={settings}>
      <UserContext.Provider value={user}>
        <ResourcesContext.Provider value={resourceManager.current}>
          <ThemeProvider theme={theme}>
            <LoadingScreenContext.Provider value={setLoadingScreenProps}>
              <NotificationBarContext.Provider value={setNotificationBarMessage}>
                <TooltipContext.Provider value={{ showTooltips, toggleTooltips }}>
                  <>
                    <LoadingScreen {...loadingScreenProps}>{children}</LoadingScreen>
                    <NotificationBar
                      message={notificationBarMessage?.message}
                      type={notificationBarMessage?.type}
                    />
                  </>
                </TooltipContext.Provider>
              </NotificationBarContext.Provider>
            </LoadingScreenContext.Provider>
          </ThemeProvider>
        </ResourcesContext.Provider>
      </UserContext.Provider>
    </SettingsContext.Provider>
  ) : null;
}
