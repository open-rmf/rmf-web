import { HotKeysEnabledProps } from 'react-hotkeys';
export interface hotKeysProps {
  openCommands: () => void;
  openRobots: () => void;
  openDoors: () => void;
  openDispensers: () => void;
  openLifts: () => void;
  openSettings: () => void;
  openOnmiPanel: () => void;
}

export const keyMap = {
  OPEN_COMMANDS: 'shift+c',
  OPEN_ROBOTS: 'shift+r',
  OPEN_DOORS: 'shift+d',
  OPEN_DISPENSERS: 'shift+i',
  OPEN_LIFTS: 'shift+l',
  OPEN_SETTINGS: 'shift+s',
  OPEN_OMNIPANEL: 'shift+o',
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
  };

  return { keyMap, handlers };
};
