import { withStyles } from '@material-ui/core/styles';

export const GlobalCss = withStyles((theme) => ({
  '@global': {
    '.MuiDivider-root': {
      backgroundColor: theme.palette.success.main,
    },
    '.MuiAccordion-root': {
      backgroundColor: theme.secondaryBackground,
      color: theme.fontColors,
    },
    '.MuiButtonBase-root': {
      color: theme.fontColors,
    },
    '.MuiTableCell-root': {
      backgroundColor: theme.mainBackground,
      color: theme.fontColors,
    },
    '.MuiToolbar-root': {
      backgroundColor: theme.mainBackground,
      color: theme.fontColors,
    },
    '.MuiPaper-root': {
      backgroundColor: theme.mainBackground,
      color: theme.fontColors,
    },
    '.MuiTableCell-head': {
      background: 'rgba(0, 0, 0, 0.1)',
      borderBottom: 'none',
    },
    '.MuiTabs-indicator': {
      backgroundColor: theme.palette.success.main,
    },
  },
}))(() => null);
