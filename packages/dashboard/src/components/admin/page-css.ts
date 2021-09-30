import { styled } from '@material-ui/core';

export const adminPageClasses = {
  pageRoot: 'admin-pages-root',
};
export const AdminPagesRoot = styled('div')(({ theme }) => ({
  [`&.${adminPageClasses.pageRoot}`]: {
    width: '84%',
    height: '100%',
    boxSizing: 'border-box',
    marginLeft: 'auto',
    padding: theme.spacing(4),
    backgroundColor: theme.palette.background.paper,
  },
}));
