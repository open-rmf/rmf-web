import { withStyles } from '@material-ui/core/styles';

export const GlobalDarkCss = withStyles(() => ({
  '@global': {
    // highlight table header in panels
    '.MuiTableCell-head': {
      borderBottom: 'none',
    },
    '#log-table .MuiTableCell-root': {
      borderBottom: '1px solid rgba(255, 255, 255, 0.5)',
    },
    '*::-webkit-scrollbar-thumb': {
      border: 'none !important',
    },
  },
}))(() => null);
