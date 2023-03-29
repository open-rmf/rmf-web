import { AppController } from '../app-contexts';

export function makeMockAppController(): AppController {
  return {
    updateSettings: jest.fn(),
    showAlert: jest.fn(),
    setExtraAppbarIcons: jest.fn(),
    updateTaskQueueTable: jest.fn(),
    forceTaskQueueTable: 0,
  };
}
