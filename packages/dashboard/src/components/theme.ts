import { createTheme } from '@material-ui/core/styles';

export const theme = createTheme({
  palette: {
    primary: {
      main: '#44497a',
      dark: '#323558',
      light: '#565d99',
    },
  },
});

// TODO - temp object to specify appBar and appDrawer values until we move to officials material version
export const customThemeValues = {
  appBar: {
    logoSize: 180,
  },
  appDrawer: {
    width: 240,
  },
};
