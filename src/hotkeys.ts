import { HotKeysEnabledProps, KeyMap, KeySequence } from 'react-hotkeys';
export interface hotKeysProps {
  openCommands: () => void;
  openDispensers: () => void;
  openDoors: () => void;
  openHelpPanel: () => void;
  openHotKeys: () => void;
  openLifts: () => void;
  openOnmiPanel: () => void;
  openRobots: () => void;
  openSettings: () => void;
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
  // Keep the same name as the KeyMap
  const handlers = {
    OPEN_COMMANDS: props.openCommands,
    OPEN_DISPENSERS: props.openDispensers,
    OPEN_DOORS: props.openDoors,
    OPEN_HELP_PANEL: props.openHelpPanel,
    OPEN_HOTKEYS: props.openHotKeys,
    OPEN_LIFTS: props.openLifts,
    OPEN_OMNIPANEL: props.openOnmiPanel,
    OPEN_ROBOTS: props.openRobots,
    OPEN_SETTINGS: props.openSettings,
  };

  return { keyMap, handlers };
};
