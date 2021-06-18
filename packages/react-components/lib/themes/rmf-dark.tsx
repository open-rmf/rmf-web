import { createMuiTheme } from '@material-ui/core/styles';
import commonTheme from './common-theme';

export const rmfDark = createMuiTheme({
  palette: {
    ...commonTheme,
    primary: {
      main: '#1F396B',
    },
    text: {
      primary: '#FBFCFF',
    },
    background: {
      default: '#103375',
      paper: '#1F396B',
    },
    divider: '#20A39E',
  },
});
