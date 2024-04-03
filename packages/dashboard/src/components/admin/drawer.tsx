import {
  Drawer,
  DrawerProps,
  List,
  ListItem,
  ListItemIcon,
  ListItemText,
  Toolbar,
  styled,
} from '@mui/material';
import { SvgIconComponent } from '@mui/icons-material';
import AccountIcon from '@mui/icons-material/AccountCircle';
import SecurityIcon from '@mui/icons-material/Security';
import React from 'react';
import { RouteProps, useNavigate, useLocation } from 'react-router';

export type AdminDrawerValues = 'Users' | 'Roles';

const drawerValuesRoutesMap: Record<AdminDrawerValues, RouteProps> = {
  Users: { path: '/users' },
  Roles: { path: '/roles' },
};

const prefix = 'drawer';
const classes = {
  drawerPaper: `${prefix}-paper`,
  drawerContainer: `${prefix}-container`,
  itemIcon: `${prefix}-itemicon`,
  activeItem: `${prefix}-active-item`,
};
const StyledDrawer = styled((props: DrawerProps) => <Drawer {...props} />)(({ theme }) => ({
  [`& .${classes.drawerPaper}`]: {
    backgroundColor: theme.palette.primary.dark,
    color: theme.palette.getContrastText(theme.palette.primary.dark),
    minWidth: 240,
    width: '16%',
  },
  [`& .${classes.drawerContainer}`]: {
    overflow: 'auto',
  },
  [`& .${classes.itemIcon}`]: {
    color: theme.palette.getContrastText(theme.palette.primary.dark),
  },
  [`& .${classes.activeItem}`]: {
    backgroundColor: `${theme.palette.primary.light} !important`,
  },
}));

export function AdminDrawer(): JSX.Element {
  const location = useLocation();
  const navigate = useNavigate();
  const activeItem = React.useMemo<AdminDrawerValues>(() => {
    const matched = Object.entries(drawerValuesRoutesMap).find(
      ([, v]) => location.pathname === `/admin${v.path}`,
    );

    return matched ? (matched[0] as AdminDrawerValues) : 'Users';
  }, [location.pathname]);

  const DrawerItem = React.useCallback(
    ({ Icon, text, route }: { Icon: SvgIconComponent; text: AdminDrawerValues; route: string }) => {
      return (
        <ListItem
          button
          className={activeItem === text ? classes.activeItem : undefined}
          onClick={() => {
            navigate(route);
          }}
        >
          <ListItemIcon>
            <Icon className={classes.itemIcon} />
          </ListItemIcon>
          <ListItemText>{text}</ListItemText>
        </ListItem>
      );
    },
    [activeItem, navigate],
  );

  return (
    <StyledDrawer variant="permanent" classes={{ paper: classes.drawerPaper }}>
      <Toolbar />
      <div className={classes.drawerContainer}>
        <List>
          <DrawerItem text="Users" route={'users'} Icon={AccountIcon} />
          <DrawerItem text="Roles" route={'roles'} Icon={SecurityIcon} />
        </List>
      </div>
    </StyledDrawer>
  );
}
