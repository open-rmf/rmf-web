import { createMuiTheme } from '@material-ui/core/styles';
import commonTheme from './common-theme';

export const rmfLight = createMuiTheme({
  //Light Grey, Cultured, Silver Sand
  palette: {
    type: 'light',
    ...commonTheme,
    primary: {
      main: '#CFD8DC ',
      light: '#ECEFF1',
      dark: '#B0BEC5',
    },
    background: {
      //Cultured, White
      default: '#EEEEEE',
      paper: '#FFFFFF',
    },
  },
});
