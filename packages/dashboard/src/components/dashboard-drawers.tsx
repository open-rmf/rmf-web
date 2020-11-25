import React from 'react';
import { saveSettings } from '../settings';
import HelpDrawer from './drawers/help-drawer';
import HotKeysDialog from './drawers/hotkeys-dialog';
import SettingsDrawer from './drawers/settings-drawer';
import { ReducerMainMenuProps } from './reducers/main-menu-reducer';

interface DashboardDrawersProps {
  reducerMainMenu: ReducerMainMenuProps;
}

export const DashboardDrawers = (props: DashboardDrawersProps): JSX.Element => {
  const { reducerMainMenu } = props;
  const {
    settings,
    showSettings,
    showHelp,
    showHotkeysDialog,
    setShowSettings,
    setSettings,
    setShowHelp,
    setShowHotkeysDialog,
    setTourState,
  } = reducerMainMenu;
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
};
