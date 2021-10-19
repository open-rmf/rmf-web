import { createMuiTheme } from '@material-ui/core/styles';
import commonTheme from './common-theme';

export const rmfLight = createMuiTheme({
  //Cultured, Gainsboro, White
  palette: {
    type: 'light',
    ...commonTheme,
    primary: {
      main: '#F5F5F5',
      dark: '#62727B',
      light: '#FFFFFF',
    },
    background: {
      //Gainsboro, White
      default: '#E0E0E0',
      paper: '#FFFFFF',
    },
  },
});
