import { styled } from '@material-ui/core';

export const adminPageClasses = {
  pageRoot: 'admin-pages-root',
  notFound: 'user-profile-page-notfound',
  manageRoles: 'user-profile-page-manageroles',
};
export const AdminPagesRoot = styled('div')(({ theme }) => ({
  [`&.${adminPageClasses.pageRoot}`]: {
    marginLeft: 'auto',
    marginRight: 'auto',
    marginTop: theme.spacing(4),
    maxWidth: 1000,
    width: '100%',
  },
  [`& .${adminPageClasses.notFound}`]: {
    marginTop: '50%',
    textAlign: 'center',
  },
  [`& .${adminPageClasses.manageRoles}`]: {
    marginTop: theme.spacing(4),
  },
}));
