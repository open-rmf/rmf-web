import { createMuiTheme } from '@material-ui/core/styles';
import commonTheme from './common-theme';

declare module '@material-ui/core/styles/createMuiTheme' {
  interface Theme {
    mapClass: string;
  }
  interface ThemeOptions {
    mapClass: string;
  }
}

export const rmfLight = createMuiTheme({
  palette: {
    ...commonTheme,
    text: {
      primary: '#1F396B',
      secondary: '#103375',
      disabled: 'rgba(0, 0, 0, 0.38)',
    },
    background: {
      default: '#F3F3F3',
      paper: '#FFFFFF',
    },
    divider: 'rgba(0, 0, 0, 0.12)',
    action: {
      active: 'rgba(0, 0, 0, 0.54)',
      hover: 'rgba(0, 0, 0, 0.04)',
      selected: 'rgba(0, 0, 0, 0.08)',
      disabledBackground: 'rgba(0, 0, 0, 0.12)',
      disabled: 'rgba(0, 0, 0, 0.26)',
    },
  },
  mapClass: '',
});
