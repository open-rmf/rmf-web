import {
  AppBar as MuiAppBar,
  IconButton,
  makeStyles,
  Menu,
  MenuItem,
  Toolbar,
  Typography,
} from '@material-ui/core';
import AccountCircleIcon from '@material-ui/icons/AccountCircle';
import DashboardIcon from '@material-ui/icons/Dashboard';
import SettingsIcon from '@material-ui/icons/Settings';
import React from 'react';
import { AuthenticatorContext, UserContext } from './auth/contexts';
import HelpIcon from '@material-ui/icons/Help';
import { MainMenuActionType } from './reducers/main-menu-reducer';

export interface AppBarProps {
  mainMenuStateHandler: any;
  // TODO: change the alarm status to required when we have an alarm
  // service working properly in the backend
  alarmState?: boolean | null;
}

export default function AppBar(props: AppBarProps): React.ReactElement {
  const { mainMenuStateHandler } = props;
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

  return (
    <MuiAppBar id="appbar" position="static">
      <Toolbar>
        <Typography variant="h6" className={classes.toolbarTitle}>
          Dashboard
        </Typography>

        <IconButton
          id="toggle-omnipanel-btn"
          color="inherit"
          onClick={() =>
            mainMenuStateHandler({
              type: MainMenuActionType.TOGGLE_OMNIPANEL,
            })
          }
        >
          <DashboardIcon />
        </IconButton>

        <IconButton
          id="show-settings-btn"
          color="inherit"
          onClick={() =>
            mainMenuStateHandler({ type: MainMenuActionType.SHOW_SETTINGS, payload: true })
          }
        >
          <SettingsIcon />
        </IconButton>
        {user && (
          <>
            <IconButton
              id="user-btn"
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
        <IconButton
          id="show-help-btn"
          color="inherit"
          onClick={() =>
            mainMenuStateHandler({ type: MainMenuActionType.SHOW_HELP, payload: true })
          }
        >
          <HelpIcon />
        </IconButton>
      </Toolbar>
    </MuiAppBar>
  );
}

const useStyles = makeStyles((_theme) => ({
  toolbarTitle: {
    flexGrow: 1,
  },
  avatar: {
    cursor: 'pointer',
  },
}));
