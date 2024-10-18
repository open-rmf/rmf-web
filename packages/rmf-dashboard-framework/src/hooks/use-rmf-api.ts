import { RmfApi } from '../services';
import { createDeferredContext } from './deferred-context';

export const [useRmfApi, RmfApiProvider] = createDeferredContext<RmfApi>();
