import { createMuiTheme } from '@material-ui/core/styles';
import commonTheme from './common-theme';

declare module '@material-ui/core/styles/createMuiTheme' {
  interface Theme {
    mainBackground: React.CSSProperties['color'];
    secondaryBackground: React.CSSProperties['color'];
    fontColors: React.CSSProperties['color'];
  }
  interface ThemeOptions {
    mainBackground: React.CSSProperties['color'];
    secondaryBackground: React.CSSProperties['color'];
    fontColors: React.CSSProperties['color'];
  }
}

export const rmfLight = createMuiTheme({
  palette: {
    ...commonTheme,
  },
  // light theme background - white
  mainBackground: '#FFFFFF',
  // light theme - snow
  secondaryBackground: '#F3F3F3',
  // dark cornflower blue
  fontColors: '#1F396B',
});
