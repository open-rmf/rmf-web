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
import appConfig from '../app-config';
import { UserContext } from '../app-contexts';

export interface AppBarProps {
  toggleShowOmniPanel(): void;
  showSettings(show: boolean): void;
}

export default function AppBar(props: AppBarProps): React.ReactElement {
  const { toggleShowOmniPanel, showSettings } = props;
  const [anchorEl, setAnchorEl] = React.useState<HTMLElement | null>(null);
  const classes = useStyles();
  const authenticator = appConfig.authenticator;
  const user = React.useContext(UserContext);

  async function handleLogout(): Promise<void> {
    await authenticator.logout();
  }

  return (
    <MuiAppBar id="appbar" position="static">
      <Toolbar>
        <Typography variant="h6" className={classes.toolbarTitle}>
          Dashboard
        </Typography>
        <IconButton color="inherit" onClick={() => toggleShowOmniPanel()}>
          <DashboardIcon />
        </IconButton>
        <IconButton color="inherit" onClick={() => showSettings(true)}>
          <SettingsIcon />
        </IconButton>
        {user && (
          <>
            <IconButton color="inherit" onClick={event => setAnchorEl(event.currentTarget)}>
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
              <MenuItem onClick={handleLogout}>Logout</MenuItem>
            </Menu>
          </>
        )}
      </Toolbar>
    </MuiAppBar>
  );
}

const useStyles = makeStyles(_theme => ({
  toolbarTitle: {
    flexGrow: 1,
  },
  avatar: {
    cursor: 'pointer',
  },
}));
