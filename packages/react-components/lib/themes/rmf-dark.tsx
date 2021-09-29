import { createMuiTheme } from '@material-ui/core/styles';
import commonTheme from './common-theme';

const base = createMuiTheme({
  palette: {
    type: 'dark',
    ...commonTheme,
    primary: {
      main: '#84693d',
      dark: '#533b11',
      light: '#bea582',
    },
    background: {
      default: '#103375',
      paper: '#2e4d83',
    },
  },
});

export const rmfDark = createMuiTheme(
  {
    '@global': {
      '.leaflet-control-zoom a': {
        color: base.palette.text.primary,
        backgroundColor: base.palette.background.paper,
      },
      '.leaflet-control-layers': {
        color: base.palette.text.primary,
        backgroundColor: base.palette.background.paper,
      },
      '.leaflet-control-layers .MuiSlider-root': {
        color: base.palette.text.primary,
      },
      '.leaflet-control-layers .MuiInputBase-input': {
        color: base.palette.text.primary,
      },
      '.leaflet-pane img': {
        filter:
          'invert(90%) sepia(12%) saturate(5773%) hue-rotate(193deg) brightness(92%) contrast(92%)',
      },
    },
  },
  base,
);
