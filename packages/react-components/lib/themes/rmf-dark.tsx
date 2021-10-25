import { createMuiTheme } from '@material-ui/core/styles';
import commonTheme from './common-theme';

const base = createMuiTheme({
  palette: {
    type: 'dark',
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
          'invert(90%) sepia(20%) saturate(120%) hue-rotate(180deg) brightness(95%) contrast(80%)',
      },
    },
  },
  base,
);
