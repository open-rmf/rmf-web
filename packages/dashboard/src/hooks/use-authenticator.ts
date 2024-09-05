import { Authenticator } from '../services/authenticator';
import { createDeferredContext } from './deferred-context';

export const [useAuthenticator, AuthenticatorProvider] = createDeferredContext<Authenticator>();
