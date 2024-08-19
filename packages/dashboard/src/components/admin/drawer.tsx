import { SvgIconComponent } from '@mui/icons-material';
import AccountIcon from '@mui/icons-material/AccountCircle';
import SecurityIcon from '@mui/icons-material/Security';
import {
  Drawer,
  List,
  ListItem,
  ListItemIcon,
  ListItemText,
  Toolbar,
  useTheme,
} from '@mui/material';
import React from 'react';
import { RouteProps, useLocation, useNavigate } from 'react-router';

export type AdminDrawerValues = 'Users' | 'Roles';

const drawerValuesRoutesMap: Record<AdminDrawerValues, RouteProps> = {
  Users: { path: '/users' },
  Roles: { path: '/roles' },
};

export function AdminDrawer(): JSX.Element {
  const location = useLocation();
  const navigate = useNavigate();
  const activeItem = React.useMemo<AdminDrawerValues>(() => {
    const matched = Object.entries(drawerValuesRoutesMap).find(
      ([, v]) => location.pathname === `/admin${v.path}`,
    );

    return matched ? (matched[0] as AdminDrawerValues) : 'Users';
  }, [location.pathname]);
  const theme = useTheme();

  const DrawerItem = React.useCallback(
    ({ Icon, text, route }: { Icon: SvgIconComponent; text: AdminDrawerValues; route: string }) => {
      return (
        <ListItem
          sx={
            activeItem === text
              ? { backgroundColor: `${theme.palette.primary.light} !important` }
              : undefined
          }
          onClick={() => {
            navigate(route);
          }}
        >
          <ListItemIcon>
            <Icon sx={{ color: theme.palette.getContrastText(theme.palette.primary.dark) }} />
          </ListItemIcon>
          <ListItemText>{text}</ListItemText>
        </ListItem>
      );
    },
    [activeItem, navigate, theme],
  );

  return (
    <Drawer
      variant="permanent"
      PaperProps={{
        sx: {
          minWidth: 240,
          backgroundColor: theme.palette.primary.dark,
          color: theme.palette.getContrastText(theme.palette.primary.dark),
        },
      }}
    >
      <Toolbar />
      <div>
        <List>
          <DrawerItem text="Users" route={'users'} Icon={AccountIcon} />
          <DrawerItem text="Roles" route={'roles'} Icon={SecurityIcon} />
        </List>
      </div>
    </Drawer>
  );
}
