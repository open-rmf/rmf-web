import { withStyles } from '@material-ui/core/styles';

export const GlobalCss = withStyles((theme) => ({
  '@global': {
    '.MuiAccordion-root': {
      backgroundColor: theme.palette.background.default,
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
    '.MuiToolbar-root': {
      backgroundColor: theme.palette.background.paper,
      color: theme.palette.text.primary,
    },
    '.MuiTableCell-head': {
      background: 'rgba(0, 0, 0, 0.1)',
      borderBottom: 'none',
    },
    '.MuiTabs-indicator': {
      backgroundColor: theme.palette.success.main,
    },
    '.MuiTabs-root': {
      backgroundColor: theme.palette.background.paper,
    },
  },
}))(() => null);
