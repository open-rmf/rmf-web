import {
  createStyles,
  IconButton,
  makeStyles,
  Menu,
  MenuItem,
  Toolbar,
  Typography,
} from '@material-ui/core';
import AccountCircleIcon from '@material-ui/icons/AccountCircle';
import SettingsIcon from '@material-ui/icons/Settings';
import React from 'react';
import { HeaderBar, LogoButton } from 'react-components';
import { AppControllerContext, ResourcesContext } from './app-contexts';
import { AuthenticatorContext, UserContext } from './auth/contexts';

const useStyles = makeStyles((theme) =>
  createStyles({
    toolbar: {
      justifyContent: 'flex-end',
      flexGrow: 4,
    },
    link: {
      textDecoration: 'none',
      color: '#000',
    },
  }),
);

export type TabValue = 'dashboard';

export interface AppBarProps {
  tabValue: TabValue;
  onTabChange?(event: React.ChangeEvent<unknown>, newValue: TabValue): void;
  // TODO: change the alarm status to required when we have an alarm
  // service working properly in the backend
  alarmState?: boolean | null;
}

export const AppBar = React.memo(
  ({ tabValue, onTabChange }: AppBarProps): React.ReactElement => {
    const { showSettings: setShowSettings } = React.useContext(AppControllerContext);
    const logoResourcesContext = React.useContext(ResourcesContext)?.logos;
    const [anchorEl, setAnchorEl] = React.useState<HTMLElement | null>(null);
    const classes = useStyles();
    const authenticator = React.useContext(AuthenticatorContext);
    const user = React.useContext(UserContext);

    async function handleLogout(): Promise<void> {
      try {
        await authenticator.logout();
      } catch (e) {
        console.error(`error logging out: ${e.message}`);
      }
    }

    const brandingIconPath = React.useMemo(() => {
      const defaultIcon = 'defaultLogo.png';
      if (!logoResourcesContext) {
        return defaultIcon;
      }
      const iconPath = logoResourcesContext.getIconPath('headerLogo');
      return iconPath ? iconPath : defaultIcon;
    }, [logoResourcesContext]);

    return (
      <div>
        <HeaderBar>
          <LogoButton logoPath={brandingIconPath} />
          <Toolbar variant="dense" className={classes.toolbar}>
            <Typography variant="caption">Powered by OpenRMF</Typography>
            <IconButton
              id="show-settings-btn"
              aria-label="settings"
              color="inherit"
              onClick={() => setShowSettings(true)}
            >
              <SettingsIcon />
            </IconButton>
            {user && (
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
                  getContentAnchorEl={null}
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
          </Toolbar>
        </HeaderBar>
      </div>
    );
  },
);

export default AppBar;
