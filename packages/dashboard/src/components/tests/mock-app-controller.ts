import { vi } from 'vitest';

import { AppController } from '../app-contexts';

export function makeMockAppController(): AppController {
  return {
    updateSettings: vi.fn(),
    showAlert: vi.fn(),
    setExtraAppbarIcons: vi.fn(),
  };
}
