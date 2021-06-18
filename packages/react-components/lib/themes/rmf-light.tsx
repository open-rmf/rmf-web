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

export const rmfLight = createMuiTheme({
  palette: {
    ...commonTheme,
    text: {
      primary: '#1F396B',
    },
    background: {
      default: '#FFFFFF',
      paper: '#F3F3F3',
    },
  },
  // light theme - snow
  secondaryBackground: '#F3F3F3',
});
