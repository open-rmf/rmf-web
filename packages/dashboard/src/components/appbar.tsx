import AccountCircleIcon from '@mui/icons-material/AccountCircle';
import HelpIcon from '@mui/icons-material/Help';
import { IconButton, Menu, MenuItem, Toolbar, Typography } from '@mui/material';
import React from 'react';
import {
  AppBarTab,
  HeaderBar,
  LogoButton,
  NavigationBar,
  Tooltip,
  useAsync,
} from 'react-components';
import { useHistory, useLocation } from 'react-router-dom';
import { UserProfileContext } from 'rmf-auth';
import { logoSize } from '../managers/resource-manager';
import {
  AdminRoute,
  CustomRoute1,
  CustomRoute2,
  DashboardRoute,
  RobotsRoute,
  TasksRoute,
} from '../util/url';
import {
  AppConfigContext,
  AppControllerContext,
  ResourcesContext,
  SettingsContext,
  TooltipsContext,
} from './app-contexts';

export type TabValue = 'infrastructure' | 'robots' | 'tasks' | 'custom1' | 'custom2' | 'admin';

function locationToTabValue(pathname: string): TabValue | undefined {
  // `DashboardRoute` being the root, it is a prefix to all routes, so we need to check exactly.
  if (pathname === DashboardRoute) return 'infrastructure';
  if (pathname.startsWith(RobotsRoute)) return 'robots';
  if (pathname.startsWith(TasksRoute)) return 'tasks';
  if (pathname.startsWith(CustomRoute1)) return 'custom1';
  if (pathname.startsWith(CustomRoute2)) return 'custom2';
  if (pathname.startsWith(AdminRoute)) return 'admin';
  return undefined;
}

export interface AppBarProps {
  extraToolbarItems?: React.ReactNode;

  // TODO: change the alarm status to required when we have an alarm
  // service working properly in the backend
  alarmState?: boolean | null;
}

export const AppBar = React.memo(({ extraToolbarItems }: AppBarProps): React.ReactElement => {
  const { showHelp: setShowHelp /* , setShowSettings */ } = React.useContext(AppControllerContext);
  const history = useHistory();
  const location = useLocation();
  const tabValue = React.useMemo(() => locationToTabValue(location.pathname), [location]);
  const logoResourcesContext = React.useContext(ResourcesContext)?.logos;
  const [anchorEl, setAnchorEl] = React.useState<HTMLElement | null>(null);
  const { authenticator } = React.useContext(AppConfigContext);
  const profile = React.useContext(UserProfileContext);
  const { showTooltips } = React.useContext(TooltipsContext);
  const safeAsync = useAsync();
  const [brandingIconPath, setBrandingIconPath] = React.useState<string>('');

  const curTheme = React.useContext(SettingsContext).themeMode;

  async function handleLogout(): Promise<void> {
    try {
      await authenticator.logout();
    } catch (e) {
      console.error(`error logging out: ${(e as Error).message}`);
    }
  }

  React.useEffect(() => {
    if (!logoResourcesContext) return;
    (async () => {
      setBrandingIconPath(await safeAsync(logoResourcesContext.getHeaderLogoPath(curTheme)));
    })();
  }, [logoResourcesContext, safeAsync, curTheme]);

  return (
    <HeaderBar>
      <LogoButton src={brandingIconPath} alt="logo" sx={{ width: logoSize }} />
      <NavigationBar value={tabValue}>
        <AppBarTab
          label="Infrastructure"
          value="infrastructure"
          aria-label="Infrastructure"
          onTabClick={() => history.push(DashboardRoute)}
        />
        <AppBarTab
          label="Robots"
          value="robots"
          aria-label="Robots"
          onTabClick={() => history.push(RobotsRoute)}
        />
        <AppBarTab
          label="Tasks"
          value="tasks"
          aria-label="Tasks"
          onTabClick={() => history.push(TasksRoute)}
        />
        <AppBarTab
          label="Custom 1"
          value="custom1"
          aria-label="Custom 1"
          onTabClick={() => history.push(CustomRoute1)}
        />
        <AppBarTab
          label="Custom 2"
          value="custom2"
          aria-label="Custom 2"
          onTabClick={() => history.push(CustomRoute2)}
        />
        {profile?.user.is_admin && (
          <AppBarTab
            label="Admin"
            value="admin"
            aria-label="Admin"
            onTabClick={() => history.push(AdminRoute)}
          />
        )}
      </NavigationBar>
      <Toolbar variant="dense" sx={{ textAlign: 'right', flexGrow: -1 }}>
        <Typography variant="caption">Powered by OpenRMF</Typography>
        {extraToolbarItems}
        {/* TODO: Hiding until we have a better theme */}
        {/* <IconButton
            id="show-settings-btn"
            aria-label="settings"
            color="inherit"
            onClick={() => setShowSettings(true)}
          >
            <SettingsIcon />
          </IconButton> */}
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
});

export default AppBar;
