import type { StyledComponent } from '@emotion/styled';
import { styled } from '@mui/material';
import type { Theme } from '@mui/material/styles';
import type { MUIStyledCommonProps } from '@mui/system';
import type React from 'react';

export const adminPageClasses = {
  pageRoot: 'admin-pages-root',
  notFound: 'user-profile-page-notfound',
  manageRoles: 'user-profile-page-manageroles',
};
export const AdminPageContainer: StyledComponent<
  MUIStyledCommonProps<Theme>,
  React.DetailedHTMLProps<React.HTMLAttributes<HTMLDivElement>, HTMLDivElement>,
  {}
> = styled('div')(({ theme }) => ({
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
