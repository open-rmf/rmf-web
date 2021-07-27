import React from 'react';
import { saveSettings, Settings } from '../settings';
import { AppControllerContext } from './app-contexts';
import SettingsDrawer from './drawers/settings-drawer';

interface AppDrawersProps {
  settings: Settings;
  showSettings: boolean;
}

export const AppDrawers = React.memo(
  (props: AppDrawersProps): JSX.Element => {
    const { settings, showSettings } = props;
    const { showSettings: setShowSettings, setSettings } = React.useContext(AppControllerContext);
    return (
      <>
        <SettingsDrawer
          settings={settings}
          open={showSettings}
          onSettingsChange={(newSettings) => {
            setSettings(newSettings);
            saveSettings(newSettings);
          }}
          onClose={() => setShowSettings(false)}
          handleCloseButton={() => setShowSettings(false)}
        />
      </>
    );
  },
);
