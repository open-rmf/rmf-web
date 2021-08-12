import { render } from '@testing-library/react';
import React from 'react';
import { User, UserContext } from '../auth/contexts';
import { createMuiTheme } from '@material-ui/core';

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
