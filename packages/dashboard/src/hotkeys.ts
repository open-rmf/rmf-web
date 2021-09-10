import { HotKeysEnabledProps, KeyMap, KeySequence } from 'react-hotkeys';
import { AppController } from './components/app-contexts';

export interface HotKeysProps {
  appController: AppController;
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

export const buildHotKeys = (props: HotKeysProps): HotKeysEnabledProps => {
  const { toggleHotkeysDialog: toggleHotkeys, toggleHelp } = props.appController;

  // Keep the same name as the KeyMap
  const handlers = {
    OPEN_HELP_PANEL: () => toggleHelp(),
    OPEN_HOTKEYS: () => toggleHotkeys(),
  };

  return { keyMap, handlers };
};
