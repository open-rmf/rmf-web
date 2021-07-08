import { act, render, RenderResult } from '@testing-library/react';
import React from 'react';
import { UserProfile, UserProfileContext } from '../auth/contexts';

/**
 * Wraps a `render` in a `act`.
 */
export async function renderAct(ui: React.ReactElement): Promise<RenderResult> {
  let root: RenderResult;
  await act(async () => {
    root = render(ui);
  });
  return root!;
}

export const superUser: UserProfile = {
  user: {
    username: 'test',
    is_admin: true,
    roles: [],
  },
  permissions: [],
};

export function mountAsUser(user: UserProfile, component: React.ReactElement) {
  return render(
    <UserProfileContext.Provider value={user}>{component}</UserProfileContext.Provider>,
  );
}
