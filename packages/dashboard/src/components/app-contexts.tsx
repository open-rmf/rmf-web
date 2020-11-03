import React from 'react';
import ResourceManager from '../resource-manager';
import { defaultSettings, Settings } from '../settings';
import { NotificationBarContext, NotificationBarProps } from './notification-bar';

/* Declares the ResourcesContext which contains the resources used on the app*/
export const ResourcesContext = React.createContext<ResourceManager | undefined>(undefined);

export const SettingsContext = React.createContext(defaultSettings());

export const TooltipContext = React.createContext({
  showTooltips: true,
  toggleTooltips: () => {},
});

export interface AppContextProviderProps extends React.PropsWithChildren<{}> {
  settings: Settings;
  resourceManager?: ResourceManager;
  notificationDispatch: React.Dispatch<React.SetStateAction<NotificationBarProps | null>>;
  tooltips: { showTooltips: boolean; toggleTooltips: () => void };
}

export function AppContextProvider(props: AppContextProviderProps): React.ReactElement {
  const { settings, notificationDispatch, resourceManager, tooltips, children } = props;
  return (
    <SettingsContext.Provider value={settings}>
      <NotificationBarContext.Provider value={notificationDispatch}>
        <TooltipContext.Provider value={tooltips}>
          <ResourcesContext.Provider value={resourceManager}>{children}</ResourcesContext.Provider>
        </TooltipContext.Provider>
      </NotificationBarContext.Provider>
    </SettingsContext.Provider>
  );
}
