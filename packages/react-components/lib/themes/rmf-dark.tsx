import { createMuiTheme } from '@material-ui/core/styles';
import commonTheme from './common-theme';

export const rmfDark = createMuiTheme({
  palette: {
    ...commonTheme,
    text: {
      primary: '#FBFCFF',
    },
    background: {
      default: '#103375',
      paper: '#1F396B',
    },
  },
});
