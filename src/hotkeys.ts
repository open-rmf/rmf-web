import { HotKeysEnabledProps, KeyMap, KeySequence } from 'react-hotkeys';
export interface hotKeysProps {
  openCommands: () => void;
  openRobots: () => void;
  openDoors: () => void;
  openDispensers: () => void;
  openLifts: () => void;
  openSettings: () => void;
  openOnmiPanel: () => void;
  openHotKeys: () => void;
}

export const keyMap: KeyMap = {
  OPEN_COMMANDS: {
    name: 'Open Commands',
    sequences: [{ sequence: 'shift+c', action: 'keypress' }],
  } as KeySequence,
  OPEN_ROBOTS: 'shift+r',
  OPEN_DOORS: 'shift+d',
  OPEN_DISPENSERS: 'shift+i',
  OPEN_LIFTS: 'shift+l',
  OPEN_SETTINGS: 'shift+s',
  OPEN_OMNIPANEL: 'shift+o',
  OPEN_HOTKEYS: 'shift+H',
};

export const buildHotKeys = (props: hotKeysProps): HotKeysEnabledProps => {
  // Keep the same name as the KeyMap
  const handlers = {
    OPEN_COMMANDS: props.openCommands,
    OPEN_ROBOTS: props.openRobots,
    OPEN_DOORS: props.openDoors,
    OPEN_DISPENSERS: props.openDispensers,
    OPEN_LIFTS: props.openLifts,
    OPEN_SETTINGS: props.openSettings,
    OPEN_OMNIPANEL: props.openOnmiPanel,
    OPEN_HOTKEYS: props.openHotKeys,
  };

  return { keyMap, handlers };
};
