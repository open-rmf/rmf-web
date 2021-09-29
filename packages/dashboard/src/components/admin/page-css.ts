import { makeStyles } from '@material-ui/core';

export const usePageStyles = makeStyles((theme) => ({
  pageRoot: {
    width: '84%',
    height: '100%',
    boxSizing: 'border-box',
    marginLeft: 'auto',
    padding: theme.spacing(4),
    backgroundColor: theme.palette.background.paper,
  },
}));
