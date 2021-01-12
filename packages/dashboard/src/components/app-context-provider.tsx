import { createMuiTheme, ThemeProvider } from '@material-ui/core';
import React from 'react';
import appConfig from '../app-config';
import ResourceManager from '../resource-manager';
import { loadSettings, SettingsContext, settingsReducer } from '../settings';
import { ResourcesContext, TooltipContext } from './app-contexts';
import { UserContext } from './auth/contexts';
import { User } from './auth/user';
import LoadingScreen, { LoadingScreenContext, LoadingScreenProps } from './loading-screen';
import NotificationBar, { NotificationBarContext, NotificationBarProps } from './notification-bar';

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
 * Wrap providers for app level contexts.
 * @param props
 */
export function AppContextProvider(props: AppContextProviderProps): JSX.Element | null {
  const { children } = props;

  const [authInitialized, setAuthInitialized] = React.useState(false);
  const [user, setUser] = React.useState<User | null>(null);

  const [settings, settingsDispatch] = React.useReducer(settingsReducer, null, () =>
    loadSettings(),
  );
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
    <SettingsContext.Provider value={[settings, settingsDispatch]}>
      <UserContext.Provider value={user}>
        <ResourcesContext.Provider value={resourceManager.current}>
          <ThemeProvider theme={theme}>
            <LoadingScreenContext.Provider value={setLoadingScreenProps}>
              <NotificationBarContext.Provider value={setNotificationBarMessage}>
                <TooltipContext.Provider value={{ showTooltips, toggleTooltips }}>
                  <React.Fragment>
                    <LoadingScreen {...loadingScreenProps}>{children}</LoadingScreen>
                    <NotificationBar
                      message={notificationBarMessage?.message}
                      type={notificationBarMessage?.type}
                    />
                  </React.Fragment>
                </TooltipContext.Provider>
              </NotificationBarContext.Provider>
            </LoadingScreenContext.Provider>
          </ThemeProvider>
        </ResourcesContext.Provider>
      </UserContext.Provider>
    </SettingsContext.Provider>
  ) : null;
}
