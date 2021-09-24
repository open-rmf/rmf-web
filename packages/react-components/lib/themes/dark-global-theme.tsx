import { withStyles } from '@material-ui/core/styles';

export const GlobalDarkCss = withStyles((theme) => ({
  '@global': {
    // highlight table header in panels
    '.MuiTableCell-head': {
      borderBottom: 'none',
    },
    '#log-table .MuiTableCell-root.MuiTableCell-body': {
      borderBottom: '1px solid rgba(255, 255, 255, 0.5)',
    },
    '#admin-user-table .MuiTableCell-root.MuiTableCell-body': {
      borderBottom: '1px solid rgba(255, 255, 255, 0.5)',
    },
    '#permission-table .MuiTableCell-root.MuiTableCell-body': {
      borderBottom: '1px solid rgba(255, 255, 255, 0.5)',
    },
    '#robot-table .MuiTableCell-root.MuiTableCell-body': {
      borderBottom: '1px solid rgba(255, 255, 255, 0.5)',
    },
    '.leaflet-control-zoom a': {
      color: theme.palette.text.primary,
      backgroundColor: theme.palette.background.paper,
    },
    '.leaflet-control-layers': {
      color: theme.palette.text.primary,
      backgroundColor: theme.palette.background.paper,
    },
    '.leaflet-control-layers .MuiSlider-root': {
      color: theme.palette.text.primary,
    },
    '.leaflet-control-layers .MuiInputBase-input': {
      color: theme.palette.text.primary,
    },
    '.leaflet-pane img': {
      filter: theme.mapClass,
    },
    '*::-webkit-scrollbar-thumb': {
      border: 'none !important',
    },
  },
}))(() => null);
