import { UserProfile } from '../services';
import { createDeferredContext } from './deferred-context';

export const [useUserProfile, UserProfileProvider] = createDeferredContext<UserProfile>();
