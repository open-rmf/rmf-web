import React from 'react';
import { saveSettings, Settings } from '../settings';
import HelpDrawer from './drawers/help-drawer';
import HotKeysDialog from './drawers/hotkeys-dialog';
import SettingsDrawer from './drawers/settings-drawer';
import { ReducerMainMenuDispatch } from './reducers/main-menu-reducer';

interface DashboardDrawersProps {
  reducerMainMenuDispatch: ReducerMainMenuDispatch;
  settings: Settings;
  showSettings: boolean;
  showHelp: boolean;
  showHotkeysDialog: boolean;
}

export const DashboardDrawers = React.memo(
  (props: DashboardDrawersProps): JSX.Element => {
    const { settings, showSettings, showHelp, showHotkeysDialog, reducerMainMenuDispatch } = props;
    const {
      setShowSettings,
      setSettings,
      setShowHelp,
      setShowHotkeysDialog,
      setTourState,
    } = reducerMainMenuDispatch;
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

        <HelpDrawer
          open={showHelp}
          handleCloseButton={() => setShowHelp(false)}
          onClose={() => setShowHelp(false)}
          setShowHotkeyDialog={() => setShowHotkeysDialog(true)}
          showTour={() => {
            setTourState(true);
            setShowHelp(false);
          }}
        />

        {showHotkeysDialog && (
          <HotKeysDialog open={showHotkeysDialog} handleClose={() => setShowHotkeysDialog(false)} />
        )}
      </>
    );
  },
);
