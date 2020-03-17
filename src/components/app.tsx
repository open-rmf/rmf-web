import { AppBar, IconButton, makeStyles, Toolbar, Typography } from '@material-ui/core/';
import { Dashboard as DashboardIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import debug from 'debug';
import React from 'react';
import 'typeface-roboto';
import DoorStateManager from '../door-state-manager';
import FleetManager from '../fleet-manager';
import LiftStateManager from '../lift-state-manager';
import './app.css';
import DoorsPanel from './doors-panel';
import LiftsPanel from './lifts-panel';
import LoadingScreen, { LoadingScreenProps } from './loading-screen';
import MainMenu from './main-menu';
import OmniPanel from './omni-panel';
import OmniPanelView from './omni-panel-view';
import PlacesPanel from './places-panel';
import RobotsPanel from './robots-panel';
import ScheduleVisualizer from './schedule-visualizer';
import { SpotlightValue } from './spotlight-expansion-panel';

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
    top: 80,
    right: '1%',
    bottom: '2%',
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

export interface AppProps {
  transportFactory: () => Promise<RomiCore.Transport>;
}

enum OmniPanelViewIndex {
  MainMenu = 0,
  Doors,
  Lifts,
  Robots,
  Places,
}

class ViewMapNode {
  constructor(public value: OmniPanelViewIndex, public parent?: ViewMapNode) {}

  addChild(view: OmniPanelViewIndex): ViewMapNode {
    return new ViewMapNode(view, this);
  }
}

type ViewMap = { [key: number]: ViewMapNode };

function makeViewMap(): ViewMap {
  const viewMap: ViewMap = {};
  const root = new ViewMapNode(OmniPanelViewIndex.MainMenu);
  viewMap[OmniPanelViewIndex.MainMenu] = root;
  viewMap[OmniPanelViewIndex.Doors] = root.addChild(OmniPanelViewIndex.Doors);
  viewMap[OmniPanelViewIndex.Lifts] = root.addChild(OmniPanelViewIndex.Lifts);
  viewMap[OmniPanelViewIndex.Robots] = root.addChild(OmniPanelViewIndex.Robots);
  viewMap[OmniPanelViewIndex.Places] = root.addChild(OmniPanelViewIndex.Places);
  return viewMap;
}

const viewMap = makeViewMap();

export default function App(props: AppProps): JSX.Element {
  const classes = useStyles();
  const [transport, setTransport] = React.useState<RomiCore.Transport | undefined>(undefined);
  const [buildingMap, setBuildingMap] = React.useState<RomiCore.BuildingMap | undefined>(undefined);

  const { current: doorStateManager } = React.useRef(
    React.useMemo(() => new DoorStateManager(), []),
  );
  const [doorStates, setDoorStates] = React.useState<Readonly<Record<string, RomiCore.DoorState>>>(
    {},
  );
  const [doors, setDoors] = React.useState<readonly RomiCore.Door[]>([]);
  const [doorSpotlight, setDoorSpotlight] = React.useState<SpotlightValue<string> | undefined>(
    undefined,
  );

  const { current: liftStateManager } = React.useRef(
    React.useMemo(() => new LiftStateManager(), []),
  );
  const [liftStates, setLiftStates] = React.useState<Readonly<Record<string, RomiCore.LiftState>>>(
    {},
  );
  const [lifts, setLifts] = React.useState<readonly RomiCore.Lift[]>([]);
  const [liftSpotlight, setLiftSpotlight] = React.useState<SpotlightValue<string> | undefined>(
    undefined,
  );

  const { current: fleetManager } = React.useRef(React.useMemo(() => new FleetManager(), []));
  const [fleets, setFleets] = React.useState(fleetManager.fleets());
  const [robotSpotlight, setRobotSpotlight] = React.useState<SpotlightValue<string> | undefined>(
    undefined,
  );

  const [placeSpotlight, setPlaceSpotlight] = React.useState<SpotlightValue<string> | undefined>(
    undefined,
  );

  const [showOmniPanel, setShowOmniPanel] = React.useState(true);
  const [currentView, setCurrentView] = React.useState(OmniPanelViewIndex.MainMenu);
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
        doorStateManager.startSubscription(x);
        liftStateManager.startSubscription(x);
        fleetManager.startSubscription(x);
        fleetManager.on('updated', () => setFleets(fleetManager.fleets()));
        setTransport(x);
      })
      .catch((e: CloseEvent) => {
        setLoading({ caption: `Unable to connect to SOSS server (${e.code})`, variant: 'error' });
      });
  }, [transportFactory, doorStateManager, liftStateManager, fleetManager]);

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

  React.useEffect(() => {
    if (currentView === OmniPanelViewIndex.Doors) {
      const listener = () => setDoorStates(doorStateManager.doorStates());
      doorStateManager.on('updated', listener);
      debug.log('started tracking door states');
      return () => {
        doorStateManager.off('updated', listener);
        debug.log('stopped tracking door states');
      };
    }
  }, [currentView, doorStateManager]);

  React.useEffect(() => {
    if (currentView === OmniPanelViewIndex.Lifts) {
      const listener = () => setLiftStates(liftStateManager.liftStates());
      liftStateManager.on('updated', listener);
      debug.log('started tracking lift states');
      return () => {
        liftStateManager.off('updated', listener);
        debug.log('stopped tracking lift states');
      };
    }
  }, [currentView, liftStateManager]);

  React.useEffect(() => {
    setDoors(buildingMap ? buildingMap.levels.flatMap(x => x.doors) : []);
    setLifts(buildingMap ? buildingMap.lifts : []);
  }, [buildingMap]);

  function handlePlaceClick(place: RomiCore.Place): void {
    setShowOmniPanel(true);
    setCurrentView(OmniPanelViewIndex.Places);
    setPlaceSpotlight({ value: place.name });
  }

  function handleRobotClick(robot: RomiCore.RobotState): void {
    setShowOmniPanel(true);
    setCurrentView(OmniPanelViewIndex.Robots);
    setRobotSpotlight({ value: robot.name });
  }

  function clearSpotlights() {
    setDoorSpotlight(undefined);
    setLiftSpotlight(undefined);
    setRobotSpotlight(undefined);
  }

  function handleClose() {
    clearSpotlights();
    setShowOmniPanel(false);
  }

  function handleBack(index: number): void {
    clearSpotlights();
    const parent = viewMap[index].parent;
    if (!parent) {
      return handleClose();
    }
    setCurrentView(parent.value);
  }

  function handleMainMenuDoorsClick(): void {
    setDoorStates(doorStateManager.doorStates());
    setCurrentView(OmniPanelViewIndex.Doors);
  }

  function handleMainMenuLiftsClick(): void {
    setLiftStates(liftStateManager.liftStates());
    setCurrentView(OmniPanelViewIndex.Lifts);
  }

  function handleMainMenuRobotsClick(): void {
    setCurrentView(OmniPanelViewIndex.Robots);
  }

  function handleMainMenuPlacesClick(): void {
    setCurrentView(OmniPanelViewIndex.Places);
  }

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
        {buildingMap && (
          <ScheduleVisualizer
            buildingMap={buildingMap}
            fleets={fleets}
            onPlaceClick={handlePlaceClick}
            onRobotClick={handleRobotClick}
          />
        )}
        {showOmniPanel && (
          <OmniPanel
            className={classes.omniPanel}
            classes={{
              backButton: classes.topLeftBorder,
              closeButton: classes.topRightBorder,
            }}
            view={currentView}
            onBack={handleBack}
            onClose={handleClose}
          >
            <OmniPanelView value={currentView} index={OmniPanelViewIndex.MainMenu}>
              <MainMenu
                onDoorsClick={handleMainMenuDoorsClick}
                onLiftsClick={handleMainMenuLiftsClick}
                onRobotsClick={handleMainMenuRobotsClick}
                onPlacesClick={handleMainMenuPlacesClick}
              />
            </OmniPanelView>
            <OmniPanelView value={currentView} index={OmniPanelViewIndex.Doors}>
              <DoorsPanel transport={transport} doorStates={doorStates} doors={doors} />
            </OmniPanelView>
            <OmniPanelView value={currentView} index={OmniPanelViewIndex.Lifts}>
              <LiftsPanel transport={transport} liftStates={liftStates} lifts={lifts} />
            </OmniPanelView>
            <OmniPanelView value={currentView} index={OmniPanelViewIndex.Robots}>
              <RobotsPanel fleets={fleets} spotlight={robotSpotlight} />
            </OmniPanelView>
            <OmniPanelView value={currentView} index={OmniPanelViewIndex.Places}>
              {buildingMap && <PlacesPanel buildingMap={buildingMap} spotlight={placeSpotlight} />}
            </OmniPanelView>
          </OmniPanel>
        )}
      </div>
    </React.Fragment>
  );
}
