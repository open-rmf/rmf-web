import { AppController } from '../app-contexts';

export function makeMockAppController(): AppController {
  return {
    showHelp: jest.fn(),
    showHotkeysDialog: jest.fn(),
    updateSettings: jest.fn(),
    showTooltips: jest.fn(),
    toggleHelp: jest.fn(),
    toggleHotkeysDialog: jest.fn(),
    toggleTooltips: jest.fn(),
    showAlert: jest.fn(),
    setExtraAppbarIcons: jest.fn(),
  };
}
