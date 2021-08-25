import { createTheme } from '@material-ui/core';

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

export const theme = createTheme({
  appBar: {
    logoSize: 180,
  },
  appDrawer: {
    width: 240,
  },
  palette: {
    primary: {
      main: '#44497a',
      dark: '#323558',
      light: '#565d99',
    },
  },
});
