import { render } from '@testing-library/react';
import React from 'react';
import { createMuiTheme } from '@material-ui/core';
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

declare module '@material-ui/core' {
  interface Theme {
    appBar: {
      logoSize: React.CSSProperties['width'];
    };
    appDrawer: {
      width: React.CSSProperties['width'];
    };
  }

  // allow configuration using `createTheme`
  interface ThemeOptions {
    appBar: {
      logoSize: React.CSSProperties['width'];
    };
    appDrawer: {
      width: React.CSSProperties['width'];
    };
  }
}

export const mockTheme = createMuiTheme({
  appBar: {
    logoSize: 180,
  },
  appDrawer: {
    width: 240,
  },
  mapClass: '',
  palette: {
    primary: {
      main: '#44497a',
      dark: '#323558',
      light: '#565d99',
    },
  },
});
