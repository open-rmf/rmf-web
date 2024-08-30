import { RmfApi } from '../services/rmf-api';
import { createDeferredContext } from './deferred-context';

export const [useRmfApi, RmfApiProvider] = createDeferredContext<RmfApi>();
