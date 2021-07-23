import { withStyles } from '@material-ui/core/styles';

export const GlobalCss = withStyles((theme) => ({
  '@global': {
    // override appbar background and font color
    '.MuiAppBar-root': {
      backgroundColor: theme.palette.background.paper,
      color: theme.palette.text.primary,
    },
    // highlight table header in panels
    '.MuiTableCell-head': {
      borderBottom: 'none',
    },
  },
}))(() => null);
