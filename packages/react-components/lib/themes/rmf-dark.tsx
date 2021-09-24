import { createMuiTheme } from '@material-ui/core/styles';
import commonTheme from './common-theme';

export const rmfDark = createMuiTheme({
  appBar: {
    logoSize: 180,
  },
  appDrawer: {
    width: 240,
  },
  palette: {
    type: 'dark',
    ...commonTheme,
    primary: {
      main: '#84693d',
      dark: '#533b11',
      light: '#bea582',
    },
    background: {
      default: '#103375',
      paper: '#2e4d83',
    },
  },
  mapClass:
    'invert(90%) sepia(12%) saturate(5773%) hue-rotate(193deg) brightness(92%) contrast(92%)',
});
