import { HotKeysEnabledProps, KeyMap, KeySequence } from 'react-hotkeys';
import { OmniPanelViewIndex } from './components/dashboard';
import { MainMenuActionType } from './components/reducers/main-menu-reducer';
export interface hotKeysProps {
  menuStateHandler: any;
  // openCommands: () => void;
  // openDispensers: () => void;
  // openDoors: () => void;
  // openHelpPanel: () => void;
  // openHotKeys: () => void;
  // openLifts: () => void;
  // openOnmiPanel: () => void;
  // openRobots: () => void;
  // openSettings: () => void;
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
  const { menuStateHandler } = props;
  const openCommands = () => {
    menuStateHandler({ type: MainMenuActionType.SHOW_OMNIPANEL, payload: true });
    menuStateHandler({
      type: MainMenuActionType.CURRENT_VIEW,
      payload: OmniPanelViewIndex.Commands,
    });
  };

  const openDispensers = () => {
    menuStateHandler({ type: MainMenuActionType.SHOW_OMNIPANEL, payload: true });
    menuStateHandler({
      type: MainMenuActionType.CURRENT_VIEW,
      payload: OmniPanelViewIndex.Dispensers,
    });
  };

  const openDoors = () => {
    menuStateHandler({ type: MainMenuActionType.SHOW_OMNIPANEL, payload: true });
    menuStateHandler({
      type: MainMenuActionType.CURRENT_VIEW,
      payload: OmniPanelViewIndex.Doors,
    });
  };

  const openLifts = () => {
    menuStateHandler({ type: MainMenuActionType.SHOW_OMNIPANEL, payload: true });
    menuStateHandler({
      type: MainMenuActionType.CURRENT_VIEW,
      payload: OmniPanelViewIndex.Lifts,
    });
  };

  const openRobots = () => {
    menuStateHandler({ type: MainMenuActionType.SHOW_OMNIPANEL, payload: true });
    menuStateHandler({
      type: MainMenuActionType.CURRENT_VIEW,
      payload: OmniPanelViewIndex.Robots,
    });
  };
  // openSettings: () => setShowSettings((prev) => !prev),
  // openOnmiPanel: () => setShowOmniPanel((prev) => !prev),
  // openHelpPanel: () => setShowHelp((prev) => !prev),
  // openHotKeys: () => setShowHotkeyDialog((prev) => !prev),

  // Keep the same name as the KeyMap
  const handlers = {
    OPEN_COMMANDS: openCommands,
    OPEN_DISPENSERS: openDispensers,
    OPEN_DOORS: openDoors,
    // OPEN_HELP_PANEL: openHelpPanel,
    // OPEN_HOTKEYS: openHotKeys,
    OPEN_LIFTS: openLifts,
    // OPEN_OMNIPANEL: openOnmiPanel,
    OPEN_ROBOTS: openRobots,
    // OPEN_SETTINGS: openSettings,
  };

  return { keyMap, handlers };
};
