import { createMuiTheme } from '@material-ui/core/styles';
import commonTheme from './common-theme';

declare module '@material-ui/core/styles/createMuiTheme' {
  interface Theme {
    secondaryBackground: React.CSSProperties['color'];
  }
  interface ThemeOptions {
    secondaryBackground: React.CSSProperties['color'];
  }
}

export const rmfDark = createMuiTheme({
  palette: {
    ...commonTheme,
    text: {
      primary: '#FBFCFF',
    },
    background: {
      default: '#1F396B',
      paper: '#103375',
    },
  },
  // dark theme - st patricks blue
  secondaryBackground: '#103375',
});
