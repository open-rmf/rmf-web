import {
  AccountCircle as AccountCircleIcon,
  Security as SecurityIcon,
  SvgIconComponent,
} from '@mui/icons-material';
import {
  Drawer,
  List,
  ListItem,
  ListItemIcon,
  ListItemText,
  Toolbar,
  useTheme,
} from '@mui/material';
import { Theme } from '@mui/material/styles';
import React from 'react';
import { RouteProps, useLocation, useNavigate } from 'react-router';

export type AdminDrawerValues = 'Users' | 'Roles';

const drawerValuesRoutesMap: Record<AdminDrawerValues, RouteProps> = {
  Users: { path: '/users' },
  Roles: { path: '/roles' },
};

interface DrawerItemProps {
  Icon: SvgIconComponent;
  text: AdminDrawerValues;
  route: string;
  activeItem: AdminDrawerValues;
  navigate: (route: string) => void;
  theme: Theme;
}

const DrawerItem = ({ Icon, text, route, activeItem, navigate, theme }: DrawerItemProps) => {
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
          <DrawerItem
            text="Users"
            route={'users'}
            Icon={AccountCircleIcon}
            activeItem={activeItem}
            navigate={navigate}
            theme={theme}
          />
          <DrawerItem
            text="Roles"
            route={'roles'}
            Icon={SecurityIcon}
            activeItem={activeItem}
            navigate={navigate}
            theme={theme}
          />
        </List>
      </div>
    </Drawer>
  );
}
