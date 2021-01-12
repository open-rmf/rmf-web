import { createMuiTheme, ThemeProvider } from '@material-ui/core';
import React from 'react';
import appConfig from '../app-config';
import ResourceManager from '../resource-manager';
import { loadSettings, saveSettings, Settings } from '../settings';
import { UserContext } from './auth/contexts';
import { User } from './auth/user';
import LoadingScreen, { LoadingScreenProps } from './loading-screen';
import NotificationBar, { NotificationBarProps } from './notification-bar';

/* Declares the ResourcesContext which contains the resources used on the app*/
export const ResourcesContext = React.createContext<ResourceManager | undefined>(undefined);

type SaveSettingsAction = {
  /**
   * Saves the provided settings into local storage and triggers a state update.
   *
   */
  type: 'saveSettings';
  settings: Settings;
};

type LoadSettingsAction = {
  /**
   * Loads the state from local storage and triggers a state update.
   */
  type: 'loadSettings';
};

export type SettingsAction = SaveSettingsAction | LoadSettingsAction;

export function settingsReducer(_state: Settings, action: SettingsAction) {
  switch (action.type) {
    case 'saveSettings':
      saveSettings(action.settings);
      return action.settings;
    case 'loadSettings':
      return loadSettings();
  }
}

/**
 * A value for this context MUST be provided, the default value is not type safe.
 */
export const SettingsContext = React.createContext<[Settings, React.Dispatch<SettingsAction>]>(
  null as any,
);

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
 * Provids all the app level contexts.
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
