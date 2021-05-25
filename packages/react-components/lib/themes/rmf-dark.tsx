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

export const rmfDark = createMuiTheme({
  palette: {
    ...commonTheme,
  },
  // dark theme background - dark cornflower blue
  mainBackground: '#1F396B',
  // dark theme - st patricks blue
  secondaryBackground: '#103375',
  // ghost white
  fontColors: '#FBFCFF',
});
