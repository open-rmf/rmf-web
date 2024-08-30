import { UserProfile } from '../services/authenticator';
import { createDeferredContext } from './deferred-context';

export const [useUserProfile, UserProfileProvider] = createDeferredContext<UserProfile>();
