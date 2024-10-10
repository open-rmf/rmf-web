import { AlertProps } from '@mui/material';

import { Settings } from '../services/settings';
import { createDeferredContext } from './deferred-context';

export interface AppController {
  updateSettings: (settings: Settings) => void;
  showAlert: (severity: AlertProps['severity'], message: string, autoHideDuration?: number) => void;
  setExtraAppbarItems: (node: React.ReactNode) => void;
}

export const [useAppController, AppControllerProvider] = createDeferredContext<AppController>();
