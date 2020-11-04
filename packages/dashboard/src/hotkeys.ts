import { HotKeysEnabledProps, KeyMap, KeySequence } from 'react-hotkeys';
import { OmniPanelViewIndex } from './components/dashboard';
import { MainMenuActionType } from './components/reducers/main-menu-reducer';
export interface hotKeysProps {
  dispatchMenu: any;
}

export const keyMap: KeyMap = {
  OPEN_COMMANDS: {
    name: 'Open Commands',
    sequences: [{ sequence: 'shift+c', action: 'keypress' }],
  } as KeySequence,
  OPEN_DISPENSERS: {
    name: 'Open Dispensers',
    sequences: [{ sequence: 'shift+i', action: 'keypress' }],
  } as KeySequence,
  OPEN_DOORS: {
    name: 'Open Doors',
    sequences: [{ sequence: 'shift+d', action: 'keypress' }],
  } as KeySequence,
  OPEN_HELP_PANEL: {
    name: 'Open Help',
    sequences: [{ sequence: 'shift+h', action: 'keypress' }],
  } as KeySequence,
  OPEN_HOTKEYS: {
    name: 'Open Hotkeys',
    sequences: [{ sequence: 'shift+?', action: 'keypress' }],
  } as KeySequence,
  OPEN_LIFTS: {
    name: 'Open Lifts',
    sequences: [{ sequence: 'shift+l', action: 'keypress' }],
  } as KeySequence,
  OPEN_OMNIPANEL: {
    name: 'Open Omnipanel',
    sequences: [{ sequence: 'shift+o', action: 'keypress' }],
  } as KeySequence,
  OPEN_ROBOTS: {
    name: 'Open Robots',
    sequences: [{ sequence: 'shift+r', action: 'keypress' }],
  } as KeySequence,
  OPEN_SETTINGS: {
    name: 'Open Settings',
    sequences: [{ sequence: 'shift+s', action: 'keypress' }],
  } as KeySequence,
};

export const buildHotKeys = (props: hotKeysProps): HotKeysEnabledProps => {
  const { dispatchMenu } = props;
  const openCommands = () => {
    dispatchMenu({ type: MainMenuActionType.SHOW_OMNIPANEL, payload: true });
    dispatchMenu({
      type: MainMenuActionType.CURRENT_VIEW,
      payload: OmniPanelViewIndex.Commands,
    });
  };

  const openDispensers = () => {
    dispatchMenu({ type: MainMenuActionType.SHOW_OMNIPANEL, payload: true });
    dispatchMenu({
      type: MainMenuActionType.CURRENT_VIEW,
      payload: OmniPanelViewIndex.Dispensers,
    });
  };

  const openDoors = () => {
    dispatchMenu({ type: MainMenuActionType.SHOW_OMNIPANEL, payload: true });
    dispatchMenu({
      type: MainMenuActionType.CURRENT_VIEW,
      payload: OmniPanelViewIndex.Doors,
    });
  };

  const openLifts = () => {
    dispatchMenu({ type: MainMenuActionType.SHOW_OMNIPANEL, payload: true });
    dispatchMenu({
      type: MainMenuActionType.CURRENT_VIEW,
      payload: OmniPanelViewIndex.Lifts,
    });
  };

  const openRobots = () => {
    dispatchMenu({ type: MainMenuActionType.SHOW_OMNIPANEL, payload: true });
    dispatchMenu({
      type: MainMenuActionType.CURRENT_VIEW,
      payload: OmniPanelViewIndex.Robots,
    });
  };

  const openOnmiPanel = () =>
    dispatchMenu({
      type: MainMenuActionType.TOGGLE_OMNIPANEL,
    });

  const openSettings = () =>
    dispatchMenu({
      type: MainMenuActionType.TOGGLE_SETTINGS,
    });

  const openHelpPanel = () =>
    dispatchMenu({
      type: MainMenuActionType.TOGGLE_HELP,
    });

  const openHotKeys = () =>
    dispatchMenu({
      type: MainMenuActionType.TOGGLE_HOTKEYS,
    });

  // Keep the same name as the KeyMap
  const handlers = {
    OPEN_COMMANDS: openCommands,
    OPEN_DISPENSERS: openDispensers,
    OPEN_DOORS: openDoors,
    OPEN_HELP_PANEL: openHelpPanel,
    OPEN_HOTKEYS: openHotKeys,
    OPEN_LIFTS: openLifts,
    OPEN_OMNIPANEL: openOnmiPanel,
    OPEN_ROBOTS: openRobots,
    OPEN_SETTINGS: openSettings,
  };

  return { keyMap, handlers };
};
