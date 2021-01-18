import {
  AppBar as MuiAppBar,
  IconButton,
  makeStyles,
  Menu,
  MenuItem,
  Toolbar,
  Typography,
  Badge,
} from '@material-ui/core';
import AccountCircleIcon from '@material-ui/icons/AccountCircle';
import DashboardIcon from '@material-ui/icons/Dashboard';
import SettingsIcon from '@material-ui/icons/Settings';
import NotificationsIcon from '@material-ui/icons/Notifications';
import React from 'react';
import { AuthenticatorContext, UserContext } from './auth/contexts';
import HelpIcon from '@material-ui/icons/Help';
import { TooltipContext } from './app-contexts';
import DashboardTooltip from 'react-components/lib/tooltip';
import { ReducerMainMenuDispatch } from './reducers/main-menu-reducer';
import FakeNotifications from '../mock/fake-notifications-manager';

export interface AppBarProps {
  reducerMainMenuDispatch: ReducerMainMenuDispatch;
  // TODO: change the alarm status to required when we have an alarm
  // service working properly in the backend
  alarmState?: boolean | null;
  countNotifications: number;
}

// temp custom hook to demo incrementing notifications
function useInterval(callback: () => void, delay: number) {
  const savedCallback = React.useRef<() => void>();

  // Remember the latest callback.
  React.useEffect(() => {
    savedCallback.current = callback;
  }, [callback]);

  // Set up the interval.
  React.useEffect(() => {
    function tick() {
      if (savedCallback && savedCallback.current) {
        savedCallback.current();
      }
    }
    if (delay !== null) {
      let id = setInterval(tick, delay);
      return () => clearInterval(id);
    }
  }, [delay]);
}
// end of temp custom hook

export const AppBar = React.memo(
  (props: AppBarProps): React.ReactElement => {
    const notificationCount = props.countNotifications;
    const {
      toggleOmnipanel,
      setShowHelp,
      setShowSettings,
      setShowNotifications,
      countNotifications,
      updateNotifications,
    } = props.reducerMainMenuDispatch;

    const [anchorEl, setAnchorEl] = React.useState<HTMLElement | null>(null);
    const classes = useStyles();
    const authenticator = React.useContext(AuthenticatorContext);
    const user = React.useContext(UserContext);
    const { showTooltips } = React.useContext(TooltipContext);

    // temp code to demonstrate notifications within the app
    let [counter, setCounter] = React.useState(0);
    const notificationsManager = new FakeNotifications();

    function notifyCallback() {
      if (counter >= notificationsManager.makeNotification().length) {
        setCounter(0);
        countNotifications(0);
        updateNotifications([]);
      } else {
        setCounter((counter += 1));
        countNotifications(notificationsManager.getNotifications(counter).length);
        updateNotifications(notificationsManager.getNotifications(counter));
      }
    }
    useInterval(notifyCallback, 5000);
    // end of temp code

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

          <DashboardTooltip
            title="View notifications from rmf"
            id="notifications-tooltip"
            enabled={showTooltips}
          >
            <IconButton
              id="notifications-btn"
              color="inherit"
              onClick={() => setShowNotifications(true)}
            >
              <Badge badgeContent={notificationCount} color="error">
                <NotificationsIcon />
              </Badge>
            </IconButton>
          </DashboardTooltip>
          <DashboardTooltip
            title="View all available panel options"
            id="omnipanel-tooltip"
            enabled={showTooltips}
          >
            <IconButton id="toggle-omnipanel-btn" color="inherit" onClick={() => toggleOmnipanel()}>
              <DashboardIcon />
            </IconButton>
          </DashboardTooltip>

          <DashboardTooltip
            title="Define dashboard trajectory settings"
            id="setting-tooltip"
            enabled={showTooltips}
          >
            <IconButton
              id="show-settings-btn"
              color="inherit"
              onClick={() => setShowSettings(true)}
            >
              <SettingsIcon />
            </IconButton>
          </DashboardTooltip>
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
          <DashboardTooltip
            title="Help tools and resources"
            id="help-tooltip"
            enabled={showTooltips}
          >
            <IconButton id="show-help-btn" color="inherit" onClick={() => setShowHelp(true)}>
              <HelpIcon />
            </IconButton>
          </DashboardTooltip>
        </Toolbar>
      </MuiAppBar>
    );
  },
);

const useStyles = makeStyles((_theme) => ({
  toolbarTitle: {
    flexGrow: 1,
  },
  avatar: {
    cursor: 'pointer',
  },
}));

export default AppBar;
