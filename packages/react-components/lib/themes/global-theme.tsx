import { withStyles } from '@material-ui/core/styles';

export const GlobalCss = withStyles((theme) => ({
  '@global': {
    // highlight table header in panels
    '.MuiTableCell-head': {
      borderBottom: 'none',
    },
  },
}))(() => null);
