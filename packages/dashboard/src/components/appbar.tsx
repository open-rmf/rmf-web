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
import { EmergencyAlarm } from './emergency-alarm';

export interface AppBarProps {
  toggleShowOmniPanel(): void;
  showSettings(show: boolean): void;
  showHelp(show: boolean): void;
  transport?: Readonly<RomiCore.Transport>;
  alarmState: boolean | null;
}

export default function AppBar(props: AppBarProps): React.ReactElement {
  const { toggleShowOmniPanel, showSettings, showHelp, transport, alarmState } = props;
  const [anchorEl, setAnchorEl] = React.useState<HTMLElement | null>(null);
  const classes = useStyles();
  const authenticator = React.useContext(AuthenticatorContext);
  const user = React.useContext(UserContext);

  const emergencyAlarmRequestPub = React.useMemo(
    () => (transport ? transport.createPublisher(RomiCore.emergency) : null),
    [transport],
  );

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

        <IconButton id="toggle-omnipanel-btn" color="inherit" onClick={() => toggleShowOmniPanel()}>
          <DashboardIcon />
        </IconButton>

        <IconButton id="show-settings-btn" color="inherit" onClick={() => showSettings(true)}>
          <SettingsIcon />
        </IconButton>
        {user && (
          <>
            <EmergencyAlarm
              isActive={alarmState}
              onTurnOn={() => emergencyAlarmRequestPub?.publish({ data: true })}
              onTurnOff={() => emergencyAlarmRequestPub?.publish({ data: false })}
            />

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
