import {
  AppBar,
  Button,
  Fade,
  IconButton,
  makeStyles,
  Toolbar,
  Typography,
  Tooltip,
} from '@material-ui/core/';
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
import RobotsPanel from './robots-panel';
import ScheduleVisualizer from './schedule-visualizer';
import SettingsDrawer from './settings-drawer';
import { SpotlightValue } from './spotlight-value';
import { DoorStateContext } from './schedule-visualizer/doors-overlay';
import { LiftStateContext } from './schedule-visualizer/lift-overlay';
import NotificationBar, { NotificationBarProps, NotificationBarContext } from './notification-bar';
import { ResourceConfigurationsType } from '../resource-manager';
import Tour from 'reactour';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { faMapSigns } from '@fortawesome/free-solid-svg-icons';
import { HighlightedWords } from './tour-step-overlay';

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
  viewMap[OmniPanelViewIndex.Dispensers] = root.addChild(OmniPanelViewIndex.Dispensers);
  viewMap[OmniPanelViewIndex.Commands] = root.addChild(OmniPanelViewIndex.Commands);
  return viewMap;
}

const viewMap = makeViewMap();

export default function App(props: AppProps): JSX.Element {
  const classes = useStyles();
  const { transportFactory, trajectoryManagerFactory, appResources } = props.appConfig;
  const [transport, setTransport] = React.useState<RomiCore.Transport | undefined>(undefined);
  const [buildingMap, setBuildingMap] = React.useState<RomiCore.BuildingMap | undefined>(undefined);
  const trajManager = React.useRef<RobotTrajectoryManager | undefined>(undefined);
  const resourceManager = React.useRef<ResourceConfigurationsType | undefined>(undefined);

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

  const [
    notificationBarMessage,
    setNotificationBarMessage,
  ] = React.useState<NotificationBarProps | null>(null);

  const [tourState, setTourState] = React.useState(true);

  const tourSteps = [
    {
      selector: '',
      content: () => (
        <>
          <Typography variant="h6">
            Welcome to the <HighlightedWords color={'#fff'}>RoMi dashboard</HighlightedWords>!
          </Typography>
        </>
      ),
      action: () => {
        setTourDefault();
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '[class="leaflet-control-zoom leaflet-bar leaflet-control"]',
      content: () => (
        <>
          <Typography variant="h6">
            Click on the <HighlightedWords>zoom buttons</HighlightedWords> to change the view of the
            floor plan. Alternatively, the scroll button on your mouse would work too!
          </Typography>
        </>
      ),
      action: () => {
        setTourDefault();
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '[class="leaflet-control-layers leaflet-control"]',
      content: () => (
        <p>
          Use the <b>floor plan button</b> to switch between available levels and enabling /
          disabling the view of different components
        </p>
      ),
      action: () => {
        setTourDefault();
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '[class="leaflet-image-layer leaflet-zoom-animated"]',
      content: () => (
        <p>
          Clicking individual components like <b>doors, robots, lifts</b> on the map will open up
          its corresponding information tab in the omnipanel.
        </p>
      ),
      action: () => {
        setTourDefault();
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '[data-name="omnipanel-button"]',
      content: () => (
        <p>
          The <b>omnipanel button</b> shows the different panel options available in the dashboard.
          Clicking each item would list different information about it!
        </p>
      ),
      action: () => {
        setTourShowOmniPanel(OmniPanelViewIndex.MainMenu);
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '[data-component="MainMenu"]',
      content: 'Each panel contains a list of the available items and their state',
      action: () => {
        setTourShowOmniPanel(OmniPanelViewIndex.MainMenu);
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '',
      content: 'Let us take a look into the Doors Panel',
      action: () => {
        setTourShowOmniPanel(OmniPanelViewIndex.Doors);
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '[data-name="main_door"]',
      content: 'Here is an example of what you will see when a door tab is expanded!',
      action: () => {
        setShowSettings(false);
        setShowOmniPanel(true);
        setCurrentView(OmniPanelViewIndex.Doors);
        if (!doorSpotlight) {
          setDoorSpotlight({ value: 'main_door' });
        }
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '',
      content:
        'The Commands Panel allows you to send different types of requests that will be handled by RoMi',
      action: () => {
        setTourShowOmniPanel(OmniPanelViewIndex.Commands);
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '[data-component="LoopForm"]',
      content: () => (
        <p>
          An example is the <b>loop request</b> which can be iterated multiple times. RoMi will
          assign the most suitable robot to perform the task at the point of request.
        </p>
      ),
      action: () => {
        setTourShowOmniPanel(OmniPanelViewIndex.Commands);
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '',
      content: () => (
        <p>
          The <b>settings button</b> opens up the drawer for different dashboard settings
        </p>
      ),
      action: () => {
        clearSpotlights();
        setShowSettings(true);
        setShowOmniPanel(false);
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '.MuiDrawer-paper',
      content: () => (
        <p>
          Finally, <b>Trajectory Animations</b> patterns can be altered to the options available.
          Look out for new features ahead!
        </p>
      ),
      action: () => {
        clearSpotlights();
        setShowOmniPanel(false);
        setShowSettings(true);
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
  ];

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
    if (!appResources) {
      return;
    }
    (async () => {
      resourceManager.current = await appResources;
    })();
  }, [appResources]);

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

  function handleMainMenuDispensersClick(): void {
    setCurrentView(OmniPanelViewIndex.Dispensers);
  }

  function handleMainMenuCommandsClick(): void {
    setCurrentView(OmniPanelViewIndex.Commands);
  }

  function setTourDefault(): void {
    clearSpotlights();
    setShowOmniPanel(false);
    setShowSettings(false);
  }

  function setTourShowOmniPanel(view: OmniPanelViewIndex): void {
    clearSpotlights();
    setShowSettings(false);
    setShowOmniPanel(true);
    setCurrentView(view);
  }

  return (
    <React.Fragment>
      <SettingsContext.Provider value={settings}>
        <NotificationBarContext.Provider value={setNotificationBarMessage}>
          {loading && <LoadingScreen {...loading} />}
          <div className={classes.container}>
            <AppBar position="static">
              <Toolbar>
                <Typography variant="h6" className={classes.toolBarTitle}>
                  Dashboard
                </Typography>
                <Tooltip title="OmniPanel">
                  <IconButton color="inherit" onClick={() => setShowOmniPanel(!showOmniPanel)}>
                    <DashboardIcon data-name="omnipanel-button" />
                  </IconButton>
                </Tooltip>
                <Tooltip title="Settings">
                  <IconButton color="inherit" onClick={() => setShowSettings(true)}>
                    <SettingsIcon />
                  </IconButton>
                </Tooltip>
                <Tooltip title="Walkthrough Guide">
                  <IconButton color="inherit" onClick={() => setTourState(true)}>
                    <FontAwesomeIcon icon={faMapSigns} />
                  </IconButton>
                </Tooltip>
              </Toolbar>
            </AppBar>
            {buildingMap && (
              <DoorStateContext.Provider value={doorStates}>
                <LiftStateContext.Provider value={liftStates}>
                  <ScheduleVisualizer
                    buildingMap={buildingMap}
                    fleets={fleets}
                    trajManager={trajManager.current}
                    appResources={resourceManager.current}
                    onDoorClick={handleDoorClick}
                    onLiftClick={handleLiftClick}
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
                <OmniPanelView id={OmniPanelViewIndex.Dispensers}>
                  <DispensersPanel
                    dispenserStates={dispenserStates}
                    spotlight={dispenserSpotlight}
                  />
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
          <NotificationBar
            message={notificationBarMessage?.message}
            type={notificationBarMessage?.type}
          />
          <Tour
            steps={tourSteps}
            isOpen={tourState}
            onRequestClose={() => {
              setTourState(false);
              clearSpotlights();
              setShowSettings(false);
              setShowOmniPanel(true);
              setCurrentView(OmniPanelViewIndex.MainMenu);
            }}
            badgeContent={(curr, tot) => `${curr} of ${tot}`}
            lastStepNextButton={<Button>Start using RoMi!</Button>}
          />
        </NotificationBarContext.Provider>
      </SettingsContext.Provider>
    </React.Fragment>
  );
}
