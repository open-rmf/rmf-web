import { createMuiTheme } from '@material-ui/core/styles';
import commonTheme from './common-theme';

const base = createMuiTheme({
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

export const rmfLight = createMuiTheme({
  overrides: {
    MuiTableCell: {
      stickyHeader: {
        backgroundColor: base.palette.primary.main,
      },
    },
  },
});
