import { Settings } from '../services';
import { createDeferredContext } from './deferred-context';

export const [useSettings, SettingsProvider] = createDeferredContext<Settings>();
