import { loadSettings } from '../../settings';
import { MainMenuState } from './main-menu-reducer';

export const mainMenuInitialValues: MainMenuState = {
  currentView: 1,
  loading: {
    caption: 'Connecting to api server...',
  },
  settings: loadSettings(),
  showHelp: false,
  showHotkeysDialog: false,
  showOmniPanel: true,
  showSettings: false,
  tourState: false,
};
