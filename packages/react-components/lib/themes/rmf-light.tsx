import { createMuiTheme } from '@material-ui/core/styles';
import commonTheme from './common-theme';

export const rmfLight = createMuiTheme({
  //Light Grey, Silver, Cultured
  palette: {
    type: 'light',
    ...commonTheme,
    primary: {
      main: '#D6D6D6 ',
      light: '#F5F5F5',
      dark: '#C3C3C3',
    },
    background: {
      //Gainsboro, White
      default: '#E0E0E0',
      paper: '#FFFFFF',
    },
  },
});
