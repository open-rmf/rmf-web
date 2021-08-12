import { createMuiTheme } from '@material-ui/core/styles';
import commonTheme from './common-theme';

export const rmfDark = createMuiTheme({
  appBar: {
    logoSize: 180,
  },
  appDrawer: {
    width: 240,
  },
  palette: {
    ...commonTheme,
    text: {
      primary: '#FBFCFF',
      secondary: '#F3F3F3',
      disabled: 'rgba(255, 255, 255, 0.5)',
    },
    background: {
      default: '#103375',
      paper: '#1F396B',
    },
    divider: 'rgba(255, 255, 255, 0.12)',
    action: {
      active: '#fff',
      hover: 'rgba(255, 255, 255, 0.08)',
      selected: 'rgba(255, 255, 255, 0.16)',
      disabledBackground: 'rgba(255, 255, 255, 0.12)',
      disabled: 'rgba(255, 255, 255, 0.3)',
    },
  },
  mapClass:
    'invert(90%) sepia(12%) saturate(5773%) hue-rotate(193deg) brightness(92%) contrast(92%)',
});
