import { createTheme } from '@mui/material/styles';
import commonTheme from './common-theme';

export const rmfLight = createTheme({
  palette: {
    mode: 'light',
    ...commonTheme,
    primary: {
      main: '#44497a',
      dark: '#323558',
      light: '#565d99',
    },
  },
});
