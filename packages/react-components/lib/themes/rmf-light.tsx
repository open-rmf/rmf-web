import { createMuiTheme } from '@material-ui/core/styles';
import commonTheme from './common-theme';

export const rmfLight = createMuiTheme({
  palette: {
    type: 'light',
    ...commonTheme,
    primary: {
      main: '#44497a',
      dark: '#323558',
      light: '#565d99',
    },
  },
});
