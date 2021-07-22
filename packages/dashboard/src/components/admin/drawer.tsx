import {
  Drawer,
  List,
  ListItem,
  ListItemIcon,
  ListItemText,
  makeStyles,
  Toolbar,
} from '@material-ui/core';
import { SvgIconComponent } from '@material-ui/icons';
import AccountIcon from '@material-ui/icons/AccountCircle';
import SecurityIcon from '@material-ui/icons/Security';
import React from 'react';
import { matchPath, RouteProps, useHistory, useLocation } from 'react-router';

export type AdminDrawerValues = 'Users' | 'Roles';

const drawerValuesRoutesMap: Record<AdminDrawerValues, RouteProps> = {
  Users: { path: '/users', exact: true },
  Roles: { path: '/roles', exact: true },
};

const useStyles = makeStyles((theme) => ({
  drawerPaper: {
    backgroundColor: theme.palette.primary.dark,
    color: theme.palette.getContrastText(theme.palette.primary.dark),
    minWidth: theme.appDrawer.width,
  },
  drawerContainer: {
    overflow: 'auto',
  },
  itemIcon: {
    color: theme.palette.getContrastText(theme.palette.primary.dark),
  },
  activeItem: {
    backgroundColor: `${theme.palette.primary.light} !important`,
  },
}));

export function AdminDrawer(): JSX.Element {
  const classes = useStyles();
  const location = useLocation();
  const history = useHistory();
  const activeItem = React.useMemo<AdminDrawerValues>(() => {
    const matched = Object.entries(drawerValuesRoutesMap).find(([_k, v]) =>
      matchPath(location.pathname, v),
    );
    return matched ? (matched[0] as AdminDrawerValues) : 'Users';
  }, [location.pathname]);

  const DrawerItem = React.useCallback(
    ({ Icon, text, route }: { Icon: SvgIconComponent; text: AdminDrawerValues; route: string }) => {
      return (
        <ListItem
          button
          className={activeItem === text ? classes.activeItem : undefined}
          onClick={() => history.push(route)}
        >
          <ListItemIcon>
            <Icon className={classes.itemIcon} />
          </ListItemIcon>
          <ListItemText>{text}</ListItemText>
        </ListItem>
      );
    },
    [activeItem, classes.activeItem, classes.itemIcon, history],
  );

  return (
    <Drawer variant="permanent" classes={{ paper: classes.drawerPaper }}>
      <Toolbar />
      <div className={classes.drawerContainer}>
        <List>
          <DrawerItem text="Users" route="/users" Icon={AccountIcon} />
          <DrawerItem text="Roles" route="/roles" Icon={SecurityIcon} />
        </List>
      </div>
    </Drawer>
  );
}
