import { act, render, RenderResult } from '@testing-library/react';
import React from 'react';
import { User, UserContext } from '../auth/contexts';

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

export const superUser: User = {
  profile: {
    username: 'test',
    is_admin: true,
    roles: [],
  },
  permissions: [],
};

export function mountAsUser(user: User, component: React.ReactElement) {
  return render(<UserContext.Provider value={user}>{component}</UserContext.Provider>);
}
