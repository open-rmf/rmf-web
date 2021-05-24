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

export const rmfLight = createMuiTheme({
  palette: {
    primary: {
      // light theme background - white
      main: '#FFFFFF',
    },
    secondary: {
      // light theme - snow
      main: '#F3F3F3',
    },
    ...commonTheme,
  },
  // dark cornflower blue
  fontColors: '#1F396B',
});
