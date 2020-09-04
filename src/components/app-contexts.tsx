import React from 'react';
import ResourceManager, { ResourceConfigurationsType } from '../resource-manager';
import { defaultSettings, Settings } from '../settings';
import { NotificationBarContext, NotificationBarProps } from './notification-bar';

/* Declares the ResourcesContext which contains the resources used on the app*/
export const ResourcesContext = React.createContext<ResourceManager>({} as ResourceManager);

export const SettingsContext = React.createContext(defaultSettings());

export interface AppContextProviderProps extends React.PropsWithChildren<{}> {
  settings: Settings;
  resources?: Partial<ResourceConfigurationsType>;
  notificationDispatch: React.Dispatch<React.SetStateAction<NotificationBarProps | null>>;
}

export function AppContextProvider(props: AppContextProviderProps): React.ReactElement {
  const { settings, notificationDispatch, resources, children } = props;
  return (
    <SettingsContext.Provider value={settings}>
      <NotificationBarContext.Provider value={notificationDispatch}>
        <ResourcesContext.Provider
          value={
            !!resources?.robots
              ? new ResourceManager(resources as ResourceConfigurationsType)
              : ({} as ResourceManager)
          }
        >
          {children}
        </ResourcesContext.Provider>
      </NotificationBarContext.Provider>
    </SettingsContext.Provider>
  );
}
