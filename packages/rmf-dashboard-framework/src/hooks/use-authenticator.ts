import { Authenticator } from '../services';
import { createDeferredContext } from './deferred-context';

export const [useAuthenticator, AuthenticatorProvider] = createDeferredContext<Authenticator>();
