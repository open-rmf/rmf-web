import { Settings } from '../services/settings';
import { createDeferredContext } from './deferred-context';

export const [useSettings, SettingsProvider] = createDeferredContext<Settings>();
