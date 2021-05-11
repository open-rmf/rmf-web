import { createMuiTheme } from '@material-ui/core/styles';

declare module '@material-ui/core/styles/createMuiTheme' {
  interface Theme {
    fontColors: {
      lightTheme: React.CSSProperties['color'];
      darkTheme: React.CSSProperties['color'];
    };
  }
  interface ThemeOptions {
    fontColors: {
      lightTheme: React.CSSProperties['color'];
      darkTheme: React.CSSProperties['color'];
    };
  }
}

export const customTheme = createMuiTheme({
  palette: {
    primary: {
      // light theme background - white
      main: '#FFFFFF',
      // dark theme background - dark cornflower blue
      dark: '#1F396B',
    },
    // used for things like sub-tabs ... etc
    secondary: {
      // light theme - snow
      main: '#F3F3F3',
      // dark theme - st patricks blue
      dark: '#103375',
    },
    success: {
      // Light sea green
      main: '#20A39E',
      light: '#4CB5B1',
      dark: '#16726E',
    },
    error: {
      // fireOpal
      main: '#F25F5C',
      dark: '#D63F3C',
      light: '#F0A0A1',
    },
    warning: {
      // indian yellow
      main: '#DEA54B',
      dark: '#D4833C',
      light: '#EBB163',
    },
    info: {
      // wisteria
      // used in admin/information buttons/indicators
      main: '#BB86FC',
      dark: '#8E44EA',
      light: '#E2CDFD',
    },
  },
  fontColors: {
    // dark cornflower blue
    lightTheme: '#1F396B',
    // ghost white
    darkTheme: '#FBFCFF',
  },
});
