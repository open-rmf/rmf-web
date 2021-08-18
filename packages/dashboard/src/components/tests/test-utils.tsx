import { render } from '@testing-library/react';
import React from 'react';
import { UserProfile, UserProfileContext } from 'rmf-auth';

export const superUser: UserProfile = {
  user: {
    username: 'test',
    is_admin: true,
    roles: [],
  },
  permissions: [],
};

export function mountAsUser(profile: UserProfile, component: React.ReactElement) {
  return render(
    <UserProfileContext.Provider value={profile}>{component}</UserProfileContext.Provider>,
  );
}
