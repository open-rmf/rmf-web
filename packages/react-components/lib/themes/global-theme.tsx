import { withStyles } from '@material-ui/core/styles';

export const GlobalCss = withStyles((theme) => ({
  '@global': {
    '.MuiDivider-root': {
      backgroundColor: theme.palette.success.main,
    },
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
      borderRight: `0.25px solid ${theme.palette.text.primary}`,
      borderLeft: `0.25px solid ${theme.palette.text.primary}`,
    },
  },
}))(() => null);
