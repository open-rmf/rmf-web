import { ThemeProvider } from '@mui/material/styles';
import { render as render_, RenderOptions as RenderOptions_ } from '@testing-library/react';
import React from 'react';
import { rmfLight } from 'react-components';
import { MemoryRouter } from 'react-router';
import { UserProfile, UserProfileContext } from 'rmf-auth';

export const superUser: UserProfile = {
  user: {
    username: 'test',
    is_admin: true,
    roles: [],
  },
  permissions: [],
};

/**
 * Provides contexts required for routing and theming.
 */
export const TestProviders: React.FC<React.PropsWithChildren> = ({ children }) => {
  return (
    <MemoryRouter>
      <ThemeProvider theme={rmfLight}>
        <UserProfileContext.Provider value={superUser}>{children}</UserProfileContext.Provider>
      </ThemeProvider>
    </MemoryRouter>
  );
};

export interface RenderOptions extends Omit<RenderOptions_, 'wrapper'> {
  profile?: UserProfile;
}

/**
 * Helper function to wrap the render function with `TestProviders`.
 */
export function render(ui: React.ReactElement, options?: RenderOptions) {
  return render_(ui, { wrapper: TestProviders, ...options });
}
