import { AppBar, IconButton, makeStyles, Toolbar, Typography } from '@material-ui/core/';
import { Dashboard as DashboardIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import 'typeface-roboto';
import DoorStateManager from '../door-state-manager';
import FleetManager from '../fleet-manager';
import LiftStateManager from '../lift-state-manager';
import './app.css';
import LoadingScreen, { LoadingScreenProps } from './loading-screen';
import OmniPanel, { OmniPanelView } from './omni-panel';
import ScheduleVisualizer from './schedule-visualizer';

const borderRadius = 20;

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

interface AppProps {
  transportFactory: () => Promise<RomiCore.Transport>;
}

export default function App(props: AppProps) {
  const classes = useStyles();
  const [transport, setTransport] = React.useState<RomiCore.Transport | undefined>(undefined);
  const [buildingMap, setBuildingMap] = React.useState<RomiCore.BuildingMap | undefined>(undefined);
  const doorStateManager = React.useRef(new DoorStateManager());
  const liftStateManager = React.useRef(new LiftStateManager());
  const fleetManager = React.useRef(new FleetManager());
  const [showOmniPanel, setShowOmniPanel] = React.useState(true);
  const [loading, setLoading] = React.useState<LoadingScreenProps | null>({
    caption: 'Connecting to SOSS...',
  });

  const transportFactory = props.transportFactory;
  React.useEffect(() => {
    setLoading({ caption: 'Connecting to SOSS server...' });
    transportFactory()
      .then(x => {
        x.on('error', console.error);
        x.once('close', () => {
          setLoading({ caption: 'Lost connection to SOSS', variant: 'error' });
          setTransport(undefined);
        });
        setTransport(x);
      })
      .catch((e: CloseEvent) => {
        setLoading({ caption: `Unable to connect to SOSS server (${e.code})`, variant: 'error' });
      });
  }, [transportFactory]);

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
            doorStateManager={doorStateManager.current}
            liftStateManager={liftStateManager.current}
            fleetManager={fleetManager.current}
            initialView={OmniPanelView.MainMenu}
            onClose={() => setShowOmniPanel(false)}
          />
        )}
      </div>
    </React.Fragment>
  );
}
