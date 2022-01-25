import { styled } from '@mui/material';

export const adminPageClasses = {
  pageRoot: 'admin-pages-root',
  notFound: 'user-profile-page-notfound',
  manageRoles: 'user-profile-page-manageroles',
};
export const AdminPageContainer = styled('div')(({ theme }) => ({
  [`&.${adminPageClasses.pageRoot}`]: {
    width: '84%',
    height: '100%',
    boxSizing: 'border-box',
    marginLeft: 'auto',
    padding: theme.spacing(4),
    backgroundColor: theme.palette.background.paper,
  },
  [`& .${adminPageClasses.notFound}`]: {
    marginTop: '50%',
    textAlign: 'center',
  },
  [`& .${adminPageClasses.manageRoles}`]: {
    marginTop: theme.spacing(4),
  },
}));
