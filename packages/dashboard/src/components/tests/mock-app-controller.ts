import { AppController } from '../app-contexts';

export function makeMockAppController(): AppController {
  return {
    showHelp: jest.fn(),
    showHotkeysDialog: jest.fn(),
    setShowSettings: jest.fn(),
    updateSettings: jest.fn(),
    showTooltips: jest.fn(),
    toggleHelp: jest.fn(),
    toggleHotkeysDialog: jest.fn(),
    toggleTooltips: jest.fn(),
    showErrorAlert: jest.fn(),
  };
}
