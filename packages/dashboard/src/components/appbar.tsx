import {
  createStyles,
  IconButton,
  makeStyles,
  Menu,
  MenuItem,
  Tab,
  Toolbar,
  Typography,
} from '@material-ui/core';
import AccountCircleIcon from '@material-ui/icons/AccountCircle';
import HelpIcon from '@material-ui/icons/Help';
import SettingsIcon from '@material-ui/icons/Settings';
import React from 'react';
import { HeaderBar, LogoButton, NavigationBar, Tooltip } from 'react-components';
import { AppControllerContext, ResourcesContext, TooltipsContext } from './app-contexts';
import { AuthenticatorContext, UserContext } from './auth/contexts';
import { RmfIngressContext, PlacesContext } from './rmf-app';
import AlertSnackBar, { iniCharger } from './alert-snack-bar';
import * as RmfModels from 'rmf-models';

const useStyles = makeStyles(() =>
  createStyles({
    toolbar: {
      textAlign: 'right',
      flexGrow: -1,
    },
  }),
);

export type TabValue = 'building' | 'robots' | 'tasks';

export interface AppBarProps {
  tabValue: TabValue;
  onTabChange?(event: React.ChangeEvent<unknown>, newValue: TabValue): void;
  // TODO: change the alarm status to required when we have an alarm
  // service working properly in the backend
  alarmState?: boolean | null;
}

export const AppBar = React.memo(
  ({ tabValue, onTabChange }: AppBarProps): React.ReactElement => {
    const { showHelp: setShowHelp, showSettings: setShowSettings } = React.useContext(
      AppControllerContext,
    );
    const logoResourcesContext = React.useContext(ResourcesContext)?.logos;
    const [anchorEl, setAnchorEl] = React.useState<HTMLElement | null>(null);
    const classes = useStyles();
    const authenticator = React.useContext(AuthenticatorContext);
    const user = React.useContext(UserContext);
    const { showTooltips } = React.useContext(TooltipsContext);

    const { sioClient } = React.useContext(RmfIngressContext) || {};
    const places = React.useContext(PlacesContext);
    const chargers = React.useMemo(
      () =>
        Object.values(places).reduce<string[]>((place, it) => {
          for (const param of it.vertex.params) {
            if (param.name === 'is_charger' && param.value_bool) {
              place.push(it.vertex.name);
              break;
            }
          }
          return place;
        }, []),
      [places],
    );

    const [charger, setCharger] = React.useState<RmfModels.ChargerRequest>(iniCharger);
    const [showAlert, setShowAlert] = React.useState(false);

    const onMessageClose = () => {
      setCharger(iniCharger);
      setShowAlert(false);
    };

    React.useEffect(() => {
      (async () => {
        if (!sioClient || chargers.length === 0) {
          return;
        }

        chargers.forEach((charger) => {
          sioClient.subscribeChargerRequest(charger, (state) => {
            setCharger(state);
          });
        });
      })();
    }, [sioClient, chargers]);

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
          <NavigationBar onTabChange={onTabChange} value={tabValue}>
            <Tab label="Building" value="building" aria-label="Building" />
            <Tab label="Tasks" value="tasks" aria-label="Tasks" />
          </NavigationBar>
          <Toolbar variant="dense" className={classes.toolbar}>
            <Typography variant="caption">Powered by OpenRMF</Typography>
            <Tooltip
              title="Define dashboard trajectory settings"
              id="setting-tooltip"
              enabled={showTooltips}
            >
              <IconButton
                id="show-settings-btn"
                aria-label="settings"
                color="inherit"
                onClick={() => setShowSettings(true)}
              >
                <SettingsIcon />
              </IconButton>
            </Tooltip>
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
        <AlertSnackBar
          message={`Robot ${charger.robot_name} has returned for charging. Please connect its charger and press ok.`}
          type={'warning'}
          charger={charger}
          showAlert={showAlert}
          onShowAlert={() => setShowAlert(true)}
          onMessageClose={onMessageClose}
        />
      </div>
    );
  },
);

export default AppBar;
