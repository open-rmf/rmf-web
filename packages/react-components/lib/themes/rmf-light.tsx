import { createMuiTheme } from '@material-ui/core/styles';
import commonTheme from './common-theme';

declare module '@material-ui/core/styles/createMuiTheme' {
  interface Theme {
    appBar: {
      logoSize: React.CSSProperties['width'];
    };
    appDrawer: {
      width: React.CSSProperties['width'];
    };
    mapClass: string;
  }
  interface ThemeOptions {
    appBar: {
      logoSize: React.CSSProperties['width'];
    };
    appDrawer: {
      width: React.CSSProperties['width'];
    };
    mapClass: string;
  }
}

export const rmfLight = createMuiTheme({
  appBar: {
    logoSize: 180,
  },
  appDrawer: {
    width: 240,
  },
  palette: {
    type: 'light',
    ...commonTheme,
    primary: {
      main: '#44497a',
      dark: '#323558',
      light: '#565d99',
    },
  },
  mapClass: '',
});
