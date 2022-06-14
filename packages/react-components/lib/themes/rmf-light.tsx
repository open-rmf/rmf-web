import { createTheme } from '@mui/material/styles';
import commonTheme from './common-theme';

export const base = createTheme({
  //Light Grey, Cultured, Silver Sand
  palette: {
    ...commonTheme,
    mode: 'light',
    primary: {
      main: '#1FD8DC ',
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
  ...base,
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
