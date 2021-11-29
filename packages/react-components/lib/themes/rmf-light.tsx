import { createTheme } from '@mui/material/styles';
import commonTheme from './common-theme';

export const base = createTheme({
  //Light Grey, Cultured, Silver Sand
  palette: {
    mode: 'light',
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

export const rmfLight = createTheme({
  components: {
    MuiTableCell: {
      styleOverrides: {
        stickyHeader: {
          backgroundColor: base.palette.primary.main,
        },
      },
    },
  },
});
