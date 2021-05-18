import { createMuiTheme } from '@material-ui/core/styles';
import commonTheme from './common-theme';

declare module '@material-ui/core/styles/createMuiTheme' {
  interface Theme {
    fontColors: React.CSSProperties['color'];
  }
  interface ThemeOptions {
    fontColors: React.CSSProperties['color'];
  }
}

export const darkTheme = createMuiTheme({
  palette: {
    primary: {
      // dark theme background - dark cornflower blue
      main: '#1F396B',
    },
    secondary: {
      // dark theme - st patricks blue
      main: '#103375',
    },
    ...commonTheme,
  },
  // ghost white
  fontColors: '#FBFCFF',
});
