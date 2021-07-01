import { withStyles } from '@material-ui/core/styles';

export const GlobalCss = withStyles((theme) => ({
  '@global': {
    '.MuiAccordion-root': {
      backgroundColor: theme.palette.background.default,
    },
    '.MuiButtonBase-root.Mui-disabled': {
      color: theme.palette.text.primary,
    },
    '.MuiAppBar-root': {
      backgroundColor: theme.palette.primary.main,
    },
    '.MuiTableCell-head': {
      background: 'rgba(0, 0, 0, 0.1)',
      borderBottom: 'none',
    },
    '.MuiFormLabel-root': {
      color: theme.palette.text.primary,
    },
    '.MuiFormLabel-root.Mui-focused': {
      color: '#A8A8A8',
    },
  },
}))(() => null);
