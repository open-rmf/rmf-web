import { AppBar, IconButton, makeStyles, Toolbar, Typography } from '@material-ui/core/';
import { Dashboard as DashboardIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { SossTransport } from '@osrf/romi-js-soss-transport';
import React from 'react';
import 'typeface-roboto';
import AppConfig from './app-config';
import './App.css';
import ScheduleVisualizer from './components/schedule-visualizer';
import LoadingScreen, { LoadingScreenProps } from './loading-screen';
import doorStates from './mock/data/door-states';
import fleets from './mock/data/fleets';
import liftStates from './mock/data/lift-states';
import { FakeAuthService } from './mock/fake-auth-service';
import OmniPanel, { OmniPanelView } from './omni-panel';

const borderRadius = 20;

const auth = new FakeAuthService(AppConfig.authUrl);

const useStyles = makeStyles(theme => ({
  container: {
    display: 'flex',
    flexFlow: 'column',
    height: '100%',
  },
  toolBarTitle: {
    flexGrow: 1,
  },
  omniPanel: {
    position: 'fixed',
    width: 350,
    top: 100,
    right: '2%',
    bottom: '3%',
    backgroundColor: theme.palette.background.default,
    zIndex: 1024,
    borderTopLeftRadius: borderRadius,
    borderTopRightRadius: borderRadius,
    boxShadow: theme.shadows[12],
  },
  topLeftBorder: {
    borderTopLeftRadius: borderRadius,
  },
  topRightBorder: {
    borderTopRightRadius: borderRadius,
  },
}));

export default function App() {
  const classes = useStyles();
  const [transport, setTransport] = React.useState<RomiCore.Transport | undefined>(undefined);
  const [buildingMap, setBuildingMap] = React.useState<RomiCore.BuildingMap | undefined>(undefined);
  const [showOmniPanel, setShowOmniPanel] = React.useState(true);
  const [loading, setLoading] = React.useState<LoadingScreenProps | null>({
    caption: 'Connecting to SOSS...',
  });

  React.useEffect(() => {
    setLoading({ caption: 'Connecting to SOSS server...' });
    SossTransport.connect('romi-dashboard', AppConfig.sossUrl, auth.token())
      .then(x => {
        x.once('error', e => console.log('error', e));
        x.once('close', () => console.log('close'));
        setTransport(x);
      })
      .catch((e: CloseEvent) => {
        setLoading({ caption: `Unable to connect to SOSS server (${e.code})`, variant: 'error' });
      });
  }, []);

  React.useEffect(() => {
    if (!transport) {
      return;
    }
    setLoading({ caption: 'Downloading building map...' });
    const request = new RomiCore.GetBuildingMap_Request();
    transport
      .call(RomiCore.getBuildingMap, request)
      .then(result => {
        setBuildingMap(result.building_map);
        setLoading(null);
      })
      .catch(() => {
        setLoading({ caption: 'Unable to download building map', variant: 'error' });
      });
  }, [transport]);

  return (
    <React.Fragment>
      {loading && <LoadingScreen {...loading} />}
      <div className={classes.container}>
        <AppBar position="static">
          <Toolbar>
            <Typography variant="h6" className={classes.toolBarTitle}>
              Dashboard
            </Typography>
            <IconButton color="inherit" onClick={() => setShowOmniPanel(true)}>
              <DashboardIcon />
            </IconButton>
          </Toolbar>
        </AppBar>
        <ScheduleVisualizer />
        {showOmniPanel && (
          <OmniPanel
            className={classes.omniPanel}
            classes={{
              backButton: classes.topLeftBorder,
              closeButton: classes.topRightBorder,
            }}
            transport={transport}
            buildingMap={buildingMap}
            doorStates={doorStates}
            liftStates={liftStates}
            fleets={fleets}
            initialView={OmniPanelView.MainMenu}
            onClose={() => setShowOmniPanel(false)}
          />
        )}
      </div>
    </React.Fragment>
  );
}
