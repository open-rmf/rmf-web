import {
  AppBar as MuiAppBar,
  IconButton,
  makeStyles,
  Menu,
  MenuItem,
  Toolbar,
  Typography,
} from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import AccountCircleIcon from '@material-ui/icons/AccountCircle';
import DashboardIcon from '@material-ui/icons/Dashboard';
import SettingsIcon from '@material-ui/icons/Settings';
import React from 'react';
import { AuthenticatorContext, UserContext } from './auth/contexts';
import HelpIcon from '@material-ui/icons/Help';
import NotificationsActiveIcon from '@material-ui/icons/NotificationsActive';
import { Alerts } from '../util/alerts';

export interface AppBarProps {
  toggleShowOmniPanel(): void;
  showSettings(show: boolean): void;
  showHelp(show: boolean): void;
  transport?: Readonly<RomiCore.Transport>;
}

class Emergency {
  static readonly typeName = 'std_msgs/msg/Bool';
  static fromObject(obj: any): Emergency {
    if (typeof obj.data !== 'boolean') throw Error('data must be of type boolean');
    return new Emergency(obj.data);
  }
  constructor(public data: boolean) {}
}

export default function AppBar(props: AppBarProps): React.ReactElement {
  const { toggleShowOmniPanel, showSettings, showHelp, transport } = props;
  const [anchorEl, setAnchorEl] = React.useState<HTMLElement | null>(null);
  const classes = useStyles();
  const authenticator = React.useContext(AuthenticatorContext);
  const user = React.useContext(UserContext);
  const [alarm, setAlarm] = React.useState(false);

  const emergency: RomiCore.RomiTopic<Emergency> = {
    validate: (msg) => Emergency.fromObject(msg),
    type: Emergency.typeName,
    topic: 'fire_alarm_trigger',
  };

  const emergencyAlarmRequestPub = React.useMemo(
    () => (transport ? transport.createPublisher(emergency) : null),
    [transport, emergency],
  );

  async function handleLogout(): Promise<void> {
    try {
      await authenticator.logout();
    } catch (e) {
      console.error(`error logging out: ${e.message}`);
    }
  }

  const handleAlarm = (): void => {
    if (alarm) {
      Alerts.verification({
        confirmCallback: () => {
          setAlarm(false);
          emergencyAlarmRequestPub?.publish({
            data: false,
          });
        },
        body: `You're about to turn off the alarm! The robots will resume their tasks.`,
      });
    } else {
      Alerts.verification({
        confirmCallback: () => {
          setAlarm(true);
          emergencyAlarmRequestPub?.publish({
            data: true,
          });
        },
        body: `You're about to fire an alarm! The robots will head to their nearest holding points. Once you accept this there is no turning back.`,
      });
    }
  };

  return (
    <MuiAppBar id="appbar" position="static">
      <Toolbar>
        <Typography variant="h6" className={classes.toolbarTitle}>
          Dashboard
        </Typography>

        <IconButton id="toggle-omnipanel-btn" color="inherit" onClick={() => toggleShowOmniPanel()}>
          <DashboardIcon />
        </IconButton>

        <IconButton id="show-settings-btn" color="inherit" onClick={() => showSettings(true)}>
          <SettingsIcon />
        </IconButton>
        {user && (
          <>
            <IconButton
              id="alarm-btn"
              color={alarm ? 'secondary' : 'inherit'}
              onClick={handleAlarm}
            >
              <NotificationsActiveIcon />
            </IconButton>
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
        <IconButton id="show-help-btn" color="inherit" onClick={() => showHelp(true)}>
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
