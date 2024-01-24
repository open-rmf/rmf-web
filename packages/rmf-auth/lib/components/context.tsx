/* istanbul ignore file */

import { Configuration, DefaultApi, Permission, User } from 'api-client';
import React from 'react';
import Authenticator from '../authenticator';

export interface UserProfile {
  user: User;
  permissions: Permission[];
}

export const UserProfileContext = React.createContext<UserProfile | null>(null);

export interface UserProfileProviderProps {
  authenticator: Authenticator;
  /**
   * Base url of rmf api server.
   */
  basePath: string;
}

export function UserProfileProvider({
  authenticator,
  basePath,
  children,
}: React.PropsWithChildren<UserProfileProviderProps>): JSX.Element {
  const [token, setToken] = React.useState(authenticator.token);
  const [userProfile, setUserProfile] = React.useState<UserProfile | null>(null);

  // TODO: The axios swagger generator is bugged and does not use this option.
  // const apiClient = new DefaultApi({
  //   accessToken: async () => {
  //     await authenticator.refreshToken();
  //     return authenticator.token || '';
  //   },
  // });
  const apiClient = React.useMemo(
    () =>
      new DefaultApi(
        new Configuration({
          basePath,
          baseOptions: {
            headers: {
              // using spread operator to avoid `Authorization: undefined` if no token is available.
              ...(token ? { Authorization: `Bearer ${token}` } : {}),
            },
          },
        }),
      ),
    [token, basePath],
  );

  React.useEffect(() => {
    const handler = () => setToken(authenticator.token);
    authenticator.on('userChanged', handler);
    return () => {
      authenticator.off('userChanged', handler);
    };
  }, [authenticator]);

  React.useEffect(() => {
    let cancel = false;
    (async () => {
      const user = (await apiClient.getUserUserGet()).data;
      const perm = (await apiClient.getEffectivePermissionsPermissionsGet()).data;
      if (cancel) return;
      setUserProfile({ user, permissions: perm });
    })();
    return () => {
      cancel = true;
    };
  }, [apiClient]);

  return <UserProfileContext.Provider value={userProfile}>{children}</UserProfileContext.Provider>;
}
