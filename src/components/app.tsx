import { AppBar, Fade, IconButton, makeStyles, Toolbar, Typography } from '@material-ui/core/';
import { Dashboard as DashboardIcon, Settings as SettingsIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import debug from 'debug';
import React from 'react';
import 'typeface-roboto';
import { AppConfig } from '../app-config';
import DispenserStateManager from '../dispenser-state-manager';
import DoorStateManager from '../door-state-manager';
import FleetManager from '../fleet-manager';
import LiftStateManager from '../lift-state-manager';
import { RobotTrajectoryManager } from '../robot-trajectory-manager';
import { loadSettings, saveSettings, Settings, SettingsContext } from '../settings';
import './app.css';
import CommandsPanel from './commands-panel';
import DispensersPanel from './dispensers-panel';
import DoorsPanel from './doors-panel';
import LiftsPanel from './lift-item/lifts-panel';
import LoadingScreen, { LoadingScreenProps } from './loading-screen';
import MainMenu from './main-menu';
import OmniPanel from './omni-panel';
import OmniPanelView from './omni-panel-view';
import PlacesPanel from './places-panel';
import RobotsPanel from './robots-panel';
import ScheduleVisualizer from './schedule-visualizer';
import SettingsDrawer from './settings-drawer';
import { SpotlightValue } from './spotlight-value';
import { DoorStateContext } from './schedule-visualizer/doors-overlay';
import { LiftStateContext } from './schedule-visualizer/lift-overlay';

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
    '@media (min-aspect-ratio: 8/10)': {
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
    '@media (max-aspect-ratio: 8/10)': {
      position: 'absolute',
      width: '100%',
      height: '35%',
      top: '65%',
      backgroundColor: theme.palette.background.default,
      zIndex: 1024,
      borderTopLeftRadius: borderRadius,
      borderTopRightRadius: borderRadius,
      boxShadow: theme.shadows[12],
    },
  },
  topLeftBorder: {
    borderTopLeftRadius: borderRadius,
  },
  topRightBorder: {
    borderTopRightRadius: borderRadius,
  },
}));

export interface AppProps {
  appConfig: AppConfig;
}

enum OmniPanelViewIndex {
  MainMenu = 0,
  Doors,
  Lifts,
  Robots,
  Places,
  Dispensers,
  Commands,
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
  viewMap[OmniPanelViewIndex.Dispensers] = root.addChild(OmniPanelViewIndex.Dispensers);
  viewMap[OmniPanelViewIndex.Commands] = root.addChild(OmniPanelViewIndex.Commands);
  return viewMap;
}

const viewMap = makeViewMap();

export default function App(props: AppProps): JSX.Element {
  const classes = useStyles();
  const { transportFactory, trajectoryManagerFactory } = props.appConfig;
  const [transport, setTransport] = React.useState<RomiCore.Transport | undefined>(undefined);
  const [buildingMap, setBuildingMap] = React.useState<RomiCore.BuildingMap | undefined>(undefined);
  const trajManager = React.useRef<RobotTrajectoryManager | undefined>(undefined);

  const doorStateManager = React.useMemo(() => new DoorStateManager(), []);
  const [doorStates, setDoorStates] = React.useState(() => doorStateManager.doorStates());
  const [doors, setDoors] = React.useState<readonly RomiCore.Door[]>([]);

  const [doorSpotlight, setDoorSpotlight] = React.useState<SpotlightValue<string> | undefined>(
    undefined,
  );

  const liftStateManager = React.useMemo(() => new LiftStateManager(), []);
  const [liftStates, setLiftStates] = React.useState(() => liftStateManager.liftStates());

  const [lifts, setLifts] = React.useState<readonly RomiCore.Lift[]>([]);
  const [liftSpotlight, setLiftSpotlight] = React.useState<SpotlightValue<string> | undefined>(
    undefined,
  );

  const fleetManager = React.useMemo(() => new FleetManager(), []);
  const [fleets, setFleets] = React.useState(fleetManager.fleets());
  const [robotSpotlight, setRobotSpotlight] = React.useState<SpotlightValue<string> | undefined>(
    undefined,
  );

  const [placeSpotlight, setPlaceSpotlight] = React.useState<SpotlightValue<string> | undefined>(
    undefined,
  );

  const dispenserStateManager = React.useMemo(() => new DispenserStateManager(), []);
  const [dispenserStates, setDispenserStates] = React.useState<
    Readonly<Record<string, RomiCore.DispenserState>>
  >({});
  const [dispenserSpotlight, setDispenserSpotlight] = React.useState<
    SpotlightValue<string> | undefined
  >(undefined);

  const [showOmniPanel, setShowOmniPanel] = React.useState(true);
  const [currentView, setCurrentView] = React.useState(OmniPanelViewIndex.MainMenu);
  const [loading, setLoading] = React.useState<LoadingScreenProps | null>({
    caption: 'Connecting to SOSS...',
  });

  const [showSettings, setShowSettings] = React.useState(false);
  const [settings, setSettings] = React.useState<Settings>(() => loadSettings());

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
        dispenserStateManager.startSubscription(x);
        liftStateManager.startSubscription(x);
        fleetManager.startSubscription(x);

        fleetManager.on('updated', () => setFleets(fleetManager.fleets()));
        liftStateManager.on('updated', () => setLiftStates(liftStateManager.liftStates()));
        doorStateManager.on('updated', () => setDoorStates(doorStateManager.doorStates()));
        setTransport(x);
      })
      .catch((e: CloseEvent) => {
        setLoading({ caption: `Unable to connect to SOSS server (${e.code})`, variant: 'error' });
      });
  }, [transportFactory, doorStateManager, liftStateManager, dispenserStateManager, fleetManager]);

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
    if (!trajectoryManagerFactory) {
      return;
    }
    (async () => {
      trajManager.current = await trajectoryManagerFactory();
    })();
  }, [trajectoryManagerFactory]);

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
    if (currentView === OmniPanelViewIndex.Dispensers) {
      const listener = () => setDispenserStates(dispenserStateManager.dispenserStates());
      dispenserStateManager.on('updated', listener);
      debug.log('started tracking dispenser states');
      return () => {
        dispenserStateManager.off('updated', listener);
        debug.log('stopped tracking dispenser states');
      };
    }
  }, [currentView, dispenserStateManager]);

  React.useEffect(() => {
    setDoors(buildingMap ? buildingMap.levels.flatMap(x => x.doors) : []);
    setLifts(buildingMap ? buildingMap.lifts : []);
  }, [buildingMap]);

  function handlePlaceClick(place: RomiCore.Place): void {
    setShowOmniPanel(true);
    setCurrentView(OmniPanelViewIndex.Places);
    setPlaceSpotlight({ value: place.name });
  }

  function handleDoorClick(door: RomiCore.Door): void {
    setShowOmniPanel(true);
    setCurrentView(OmniPanelViewIndex.Doors);
    setDoorSpotlight({ value: door.name });
  }

  function handleRobotClick(robot: RomiCore.RobotState): void {
    setShowOmniPanel(true);
    setCurrentView(OmniPanelViewIndex.Robots);
    setRobotSpotlight({ value: robot.name });
  }

  function handleLiftClick(lift: RomiCore.Lift): void {
    setShowOmniPanel(true);
    setCurrentView(OmniPanelViewIndex.Lifts);
    setLiftSpotlight({ value: lift.name });
  }

  function clearSpotlights() {
    setDoorSpotlight(undefined);
    setLiftSpotlight(undefined);
    setRobotSpotlight(undefined);
    setDispenserSpotlight(undefined);
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

  function handleMainMenuDispensersClick(): void {
    setCurrentView(OmniPanelViewIndex.Dispensers);
  }

  function handleMainMenuCommandsClick(): void {
    setCurrentView(OmniPanelViewIndex.Commands);
  }

  return (
    <React.Fragment>
      <SettingsContext.Provider value={settings}>
        {loading && <LoadingScreen {...loading} />}
        <div className={classes.container}>
          <AppBar position="static">
            <Toolbar>
              <Typography variant="h6" className={classes.toolBarTitle}>
                Dashboard
              </Typography>
              <IconButton color="inherit" onClick={() => setShowOmniPanel(!showOmniPanel)}>
                <DashboardIcon />
              </IconButton>
              <IconButton color="inherit" onClick={() => setShowSettings(true)}>
                <SettingsIcon />
              </IconButton>
            </Toolbar>
          </AppBar>
          {buildingMap && (
            <DoorStateContext.Provider value={doorStates}>
              <LiftStateContext.Provider value={liftStates}>
                <ScheduleVisualizer
                  buildingMap={buildingMap}
                  fleets={fleets}
                  trajManager={trajManager.current}
                  onDoorClick={handleDoorClick}
                  onLiftClick={handleLiftClick}
                  onPlaceClick={handlePlaceClick}
                  onRobotClick={handleRobotClick}
                />
              </LiftStateContext.Provider>
            </DoorStateContext.Provider>
          )}
          <Fade in={showOmniPanel}>
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
              <OmniPanelView id={OmniPanelViewIndex.MainMenu}>
                <MainMenu
                  onDoorsClick={handleMainMenuDoorsClick}
                  onLiftsClick={handleMainMenuLiftsClick}
                  onRobotsClick={handleMainMenuRobotsClick}
                  onPlacesClick={handleMainMenuPlacesClick}
                  onDispensersClick={handleMainMenuDispensersClick}
                  onCommandsClick={handleMainMenuCommandsClick}
                />
              </OmniPanelView>
              <OmniPanelView id={OmniPanelViewIndex.Doors}>
                <DoorsPanel
                  transport={transport}
                  doorStates={doorStates}
                  doors={doors}
                  spotlight={doorSpotlight}
                />
              </OmniPanelView>
              <OmniPanelView id={OmniPanelViewIndex.Lifts}>
                <LiftsPanel
                  transport={transport}
                  liftStates={liftStates}
                  lifts={lifts}
                  spotlight={liftSpotlight}
                />
              </OmniPanelView>
              <OmniPanelView id={OmniPanelViewIndex.Robots}>
                <RobotsPanel fleets={fleets} spotlight={robotSpotlight} />
              </OmniPanelView>
              <OmniPanelView id={OmniPanelViewIndex.Places}>
                {buildingMap && (
                  <PlacesPanel buildingMap={buildingMap} spotlight={placeSpotlight} />
                )}
              </OmniPanelView>
              <OmniPanelView id={OmniPanelViewIndex.Dispensers}>
                <DispensersPanel dispenserStates={dispenserStates} spotlight={dispenserSpotlight} />
              </OmniPanelView>
              <OmniPanelView id={OmniPanelViewIndex.Commands}>
                <CommandsPanel transport={transport} fleets={fleets} />
              </OmniPanelView>
            </OmniPanel>
          </Fade>
          <SettingsDrawer
            settings={settings}
            open={showSettings}
            onSettingsChange={newSettings => {
              setSettings(newSettings);
              saveSettings(newSettings);
            }}
            onClose={() => setShowSettings(false)}
          />
        </div>
      </SettingsContext.Provider>
    </React.Fragment>
  );
}
