import { withStyles } from '@material-ui/core/styles';

export const GlobalDarkCss = withStyles(() => ({
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
    '*::-webkit-scrollbar-thumb': {
      border: 'none !important',
    },
  },
}))(() => null);
