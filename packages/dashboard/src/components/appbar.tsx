import { IconButton, Menu, MenuItem, Tab, Toolbar, Typography } from '@material-ui/core';
import { makeStyles, createStyles } from '@material-ui/styles';
import AccountCircleIcon from '@material-ui/icons/AccountCircle';
import HelpIcon from '@material-ui/icons/Help';
import React from 'react';
import { HeaderBar, LogoButton, NavigationBar, Tooltip, useAsync } from 'react-components';
import { useHistory, useLocation } from 'react-router-dom';
import { UserProfileContext } from 'rmf-auth';
import { AdminRoute, DashboardRoute, RobotsRoute, TasksRoute } from '../util/url';
import {
  AppConfigContext,
  AppControllerContext,
  ResourcesContext,
  TooltipsContext,
} from './app-contexts';
import { customThemeValues } from './theme';

const useStyles = makeStyles((theme) =>
  createStyles({
    appBar: {
      zIndex: theme.zIndex.drawer + 1,
    },
    logoBtn: {
      width: customThemeValues.appBar.logoSize,
    },
    toolbar: {
      textAlign: 'right',
      flexGrow: -1,
    },
  }),
);

export type TabValue = 'building' | 'robots' | 'tasks' | 'admin';

function locationToTabValue(pathname: string): TabValue | undefined {
  // `DashboardRoute` being the root, it is a prefix to all routes, so we need to check exactly.
  if (pathname === DashboardRoute) return 'building';
  if (pathname.startsWith(TasksRoute)) return 'tasks';
  if (pathname.startsWith(RobotsRoute)) return 'robots';
  if (pathname.startsWith(AdminRoute)) return 'admin';
  return undefined;
}

export interface AppBarProps {
  // TODO: change the alarm status to required when we have an alarm
  // service working properly in the backend
  alarmState?: boolean | null;
}

export const AppBar = React.memo(
  (): React.ReactElement => {
    const { showHelp: setShowHelp } = React.useContext(AppControllerContext);
    const history = useHistory();
    const location = useLocation();
    const tabValue = React.useMemo(() => locationToTabValue(location.pathname), [location]);
    const logoResourcesContext = React.useContext(ResourcesContext)?.logos;
    const [anchorEl, setAnchorEl] = React.useState<HTMLElement | null>(null);
    const classes = useStyles();
    const { authenticator } = React.useContext(AppConfigContext);
    const profile = React.useContext(UserProfileContext);
    const { showTooltips } = React.useContext(TooltipsContext);
    const safeAsync = useAsync();
    const [brandingIconPath, setBrandingIconPath] = React.useState<string>('');

    async function handleLogout(): Promise<void> {
      try {
        await authenticator.logout();
      } catch (e) {
        console.error(`error logging out: ${e.message}`);
      }
    }

    React.useLayoutEffect(() => {
      if (!logoResourcesContext) return;
      (async () => {
        setBrandingIconPath(await safeAsync(logoResourcesContext.getHeaderLogoPath()));
      })();
    }, [logoResourcesContext, safeAsync]);

    return (
      <HeaderBar className={classes.appBar}>
        <LogoButton src={brandingIconPath} alt="logo" className={classes.logoBtn} />
        <NavigationBar value={tabValue}>
          <Tab
            label="Building"
            value="building"
            aria-label="Building"
            onClick={() => history.push(DashboardRoute)}
          />
          <Tab
            label="Robots"
            value="robots"
            aria-label="Robots"
            onClick={() => history.push(RobotsRoute)}
          />
          <Tab
            label="Tasks"
            value="tasks"
            aria-label="Tasks"
            onClick={() => history.push(TasksRoute)}
          />
          {profile?.user.is_admin && (
            <Tab
              label="Admin"
              value="admin"
              aria-label="Admin"
              onClick={() => history.push(AdminRoute)}
            />
          )}
        </NavigationBar>
        <Toolbar variant="dense" className={classes.toolbar}>
          <Typography variant="caption">Powered by OpenRMF</Typography>
          {profile && (
            <>
              <IconButton
                id="user-btn"
                aria-label={'user-btn'}
                color="inherit"
                onClick={(event) => setAnchorEl(event.currentTarget)}
              >
                <AccountCircleIcon />
              </IconButton>
              <Menu
                anchorEl={anchorEl}
                anchorOrigin={{
                  vertical: 'bottom',
                  horizontal: 'right',
                }}
                transformOrigin={{
                  vertical: 'top',
                  horizontal: 'right',
                }}
                open={!!anchorEl}
                onClose={() => setAnchorEl(null)}
              >
                <MenuItem id="logout-btn" onClick={handleLogout}>
                  Logout
                </MenuItem>
              </Menu>
            </>
          )}
          <Tooltip title="Help tools and resources" id="help-tooltip" enabled={showTooltips}>
            <IconButton
              id="show-help-btn"
              aria-label="help"
              color="inherit"
              onClick={() => setShowHelp(true)}
            >
              <HelpIcon />
            </IconButton>
          </Tooltip>
        </Toolbar>
      </HeaderBar>
    );
  },
);

export default AppBar;
