import React from 'react';
import { saveSettings } from '../settings';
import HelpDrawer from './drawers/help-drawer';
import HotKeysDialog from './drawers/hotkeys-dialog';
import SettingsDrawer from './drawers/settings-drawer';
import { MainMenuAction, MainMenuActionType, MainMenuState } from './reducers/main-menu-reducer';

interface DashboardDrawersProps {
  stateMenu: MainMenuState;
  dispatchMenu: React.Dispatch<MainMenuAction>;
}

export const DashboardDrawers = (props: DashboardDrawersProps): JSX.Element => {
  const { stateMenu, dispatchMenu } = props;
  return (
    <>
      <SettingsDrawer
        settings={stateMenu.settings}
        open={stateMenu.showSettings}
        onSettingsChange={(newSettings) => {
          dispatchMenu({ type: MainMenuActionType.Settings, payload: newSettings });
          saveSettings(newSettings);
        }}
        onClose={() => dispatchMenu({ type: MainMenuActionType.ShowSettings, payload: false })}
        handleCloseButton={() =>
          dispatchMenu({ type: MainMenuActionType.ShowSettings, payload: false })
        }
      />

      <HelpDrawer
        open={stateMenu.showHelp}
        handleCloseButton={() =>
          dispatchMenu({ type: MainMenuActionType.ShowHelp, payload: false })
        }
        onClose={() => dispatchMenu({ type: MainMenuActionType.ShowHelp, payload: false })}
        setShowHotkeyDialog={() =>
          dispatchMenu({
            type: MainMenuActionType.ShowHotkeysDialog,
            payload: true,
          })
        }
        showTour={() => {
          dispatchMenu({ type: MainMenuActionType.TourState, payload: true });
          dispatchMenu({ type: MainMenuActionType.ShowHelp, payload: false });
        }}
      />

      {stateMenu.showHotkeysDialog && (
        <HotKeysDialog
          open={stateMenu.showHotkeysDialog}
          handleClose={() =>
            dispatchMenu({
              type: MainMenuActionType.ShowHotkeysDialog,
              payload: false,
            })
          }
        />
      )}
    </>
  );
};
