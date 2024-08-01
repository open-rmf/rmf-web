import { createTheme } from '@mui/material/styles';

import commonTheme from './common-theme';

const base = createTheme({
  palette: {
    mode: 'dark',
    ...commonTheme,
    primary: {
      //Charcoal, Rich Black Fogra 29, Cadet
      main: '#37474F',
      dark: '#102027',
      light: '#62727B',
    },
    background: {
      //Rich Black Fogra 29, Cadet
      default: '#102027',
      paper: '#62727B',
    },
  },
});

export const rmfDark = createTheme(
  {
    components: {
      MuiTableCell: {
        styleOverrides: {
          stickyHeader: {
            backgroundColor: base.palette.primary.main,
          },
        },
      },
    },
  },
  base,
);
