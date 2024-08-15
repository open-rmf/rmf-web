import React from 'react';

import { useNonNullableContext } from '../hooks';
import { UserProfile } from '../services/authenticator';
import { RmfApiContext } from './rmf-dashboard';

export const UserProfileContext = React.createContext<UserProfile | null>(null);

export function UserProfileProvider({ children }: React.PropsWithChildren<{}>): JSX.Element {
  const [userProfile, setUserProfile] = React.useState<UserProfile | null>(null);
  const rmfApi = useNonNullableContext(RmfApiContext);

  React.useEffect(() => {
    let cancel = false;
    (async () => {
      const user = (await rmfApi.defaultApi.getUserUserGet()).data;
      const perm = (await rmfApi.defaultApi.getEffectivePermissionsPermissionsGet()).data;
      if (cancel) return;
      setUserProfile({ user, permissions: perm });
    })();
    return () => {
      cancel = true;
    };
  }, [rmfApi]);

  return (
    <UserProfileContext.Provider value={userProfile}>
      {userProfile && children}
    </UserProfileContext.Provider>
  );
}
