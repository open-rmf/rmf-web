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
import React from 'react';
import { HeaderBar, LogoButton } from 'react-components';
import { AppConfigContext, ResourcesContext } from './app-contexts';
import { UserContext } from './auth/contexts';

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
    logoBtn: {
      width: theme.appBar.logoSize,
    },
  }),
);

export interface AppBarProps {
  // TODO: change the alarm status to required when we have an alarm
  // service working properly in the backend
  alarmState?: boolean | null;
}

export const AppBar = React.memo(
  (): React.ReactElement => {
    const logoResourcesContext = React.useContext(ResourcesContext)?.logos;
    const [anchorEl, setAnchorEl] = React.useState<HTMLElement | null>(null);
    const classes = useStyles();
    const { authenticator } = React.useContext(AppConfigContext);
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
          <LogoButton src={brandingIconPath} alt="logo" className={classes.logoBtn} />
          <Toolbar variant="dense" className={classes.toolbar}>
            <Typography variant="caption">Powered by OpenRMF</Typography>
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
