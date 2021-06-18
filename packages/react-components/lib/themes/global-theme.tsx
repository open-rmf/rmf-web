import { withStyles } from '@material-ui/core/styles';

export const GlobalCss = withStyles((theme) => ({
  '@global': {
    '.MuiDivider-root': {
      backgroundColor: theme.palette.success.main,
    },
    '.MuiAccordion-root': {
      backgroundColor: theme.palette.background.paper,
    },
    '.MuiSvgIcon-root': {
      color: theme.palette.text.primary,
    },
    '.MuiButtonBase-root': {
      color: theme.palette.text.primary,
    },
    '.MuiButtonBase-root.Mui-disabled': {
      color: theme.palette.text.primary,
    },
    '.MuiTableCell-root': {
      backgroundColor: theme.palette.background.default,
    },
    '.MuiToolbar-root': {
      backgroundColor: theme.palette.background.default,
    },
    '.MuiPaper-root': {
      backgroundColor: theme.palette.background.default,
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
