import { Fade, makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import 'typeface-roboto';
import appConfig from '../app-config';
import DispenserStateManager from '../dispenser-state-manager';
import DoorStateManager from '../door-state-manager';
import FleetManager from '../fleet-manager';
import LiftStateManager from '../lift-state-manager';
import NegotiationStatusManager from '../negotiation-status-manager';
import { ResourceConfigurationsType } from '../resource-manager';
import { RobotTrajectoryManager, NegotiationTrajectoryResponse } from '../robot-trajectory-manager';
import { loadSettings, saveSettings, Settings } from '../settings';
import { AppContextProvider } from './app-contexts';
import AppBar from './appbar';
import CommandsPanel from './commands-panel';
import DispensersPanel from './dispensers-panel';
import DoorsPanel from './doors-panel';
import LiftsPanel from './lift-item/lifts-panel';
import NegotiationsPanel from './negotiations-panel';
import LoadingScreen, { LoadingScreenProps } from './loading-screen';
import MainMenu from './main-menu';
import NotificationBar, { NotificationBarProps } from './notification-bar';
import OmniPanel from './omni-panel';
import OmniPanelView from './omni-panel-view';
import { RmfContextProvider } from './rmf-contexts';
import RobotsPanel from './robots-panel';
import ScheduleVisualizer from './schedule-visualizer';
import SettingsDrawer from './settings-drawer';
import { SpotlightValue } from './spotlight-value';

const debug = Debug('App');
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
      // put it above leaflet panes, https://leafletjs.com/reference-1.6.0.html#map-pane
      zIndex: 610,
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
      // put it above leaflet panes, https://leafletjs.com/reference-1.6.0.html#map-pane
      zIndex: 610,
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

enum OmniPanelViewIndex {
  MainMenu = 0,
  Doors,
  Lifts,
  Robots,
  Dispensers,
  Commands,
  Negotiations,
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
  viewMap[OmniPanelViewIndex.Negotiations] = root.addChild(OmniPanelViewIndex.Negotiations);
  return viewMap;
}

const viewMap = makeViewMap();

export default function Dashboard(_props: {}): React.ReactElement {
  debug('render');

  const { appResources, transportFactory, trajectoryManagerFactory, trajServerUrl } = appConfig;
  const classes = useStyles();
  const [transport, setTransport] = React.useState<RomiCore.Transport | undefined>(undefined);
  const [buildingMap, setBuildingMap] = React.useState<RomiCore.BuildingMap | undefined>(undefined);
  const trajManager = React.useRef<RobotTrajectoryManager | undefined>(undefined);
  const resourceManager = React.useRef<ResourceConfigurationsType | undefined>(undefined);

  const doorStateManager = React.useMemo(() => new DoorStateManager(), []);
  const [doorStates, setDoorStates] = React.useState(() => doorStateManager.doorStates());
  const [doors, setDoors] = React.useState<RomiCore.Door[]>([]);
  const [doorSpotlight, setDoorSpotlight] = React.useState<SpotlightValue<string> | undefined>(
    undefined,
  );

  const liftStateManager = React.useMemo(() => new LiftStateManager(), []);
  const [liftStates, setLiftStates] = React.useState(() => liftStateManager.liftStates());
  const [lifts, setLifts] = React.useState<RomiCore.Lift[]>([]);
  const [liftSpotlight, setLiftSpotlight] = React.useState<SpotlightValue<string> | undefined>(
    undefined,
  );

  const fleetManager = React.useMemo(() => new FleetManager(), []);
  const [fleets, setFleets] = React.useState(fleetManager.fleets());
  const fleetNames = React.useRef<string[]>([]);
  const [robotSpotlight, setRobotSpotlight] = React.useState<SpotlightValue<string> | undefined>(
    undefined,
  );
  const newFleetNames = fleets.map(fleet => fleet.name);
  if (newFleetNames.some(fleetName => !fleetNames.current.includes(fleetName))) {
    fleetNames.current = newFleetNames;
  }

  const dispenserStateManager = React.useMemo(() => new DispenserStateManager(), []);
  const [dispenserStates, setDispenserStates] = React.useState<
    Readonly<Record<string, RomiCore.DispenserState>>
  >({});
  const [dispenserSpotlight, setDispenserSpotlight] = React.useState<
    SpotlightValue<string> | undefined
  >(undefined);

  const negotiationStatusManager = React.useMemo(
    () => new NegotiationStatusManager(trajServerUrl),
    [trajServerUrl],
  );
  const [negotiationSpotlight, setNegotiationSpotlight] = React.useState<
    SpotlightValue<string> | undefined
  >(undefined);
  const [negotiationStatus, setNegotiationStatus] = React.useState(
    negotiationStatusManager.allConflicts(),
  );
  const [negotiationTrajStore, setNegotiationTrajStore] = React.useState<
    Record<string, NegotiationTrajectoryResponse>
  >({});

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
        negotiationStatusManager.startSubscription();

        fleetManager.on('updated', () => setFleets(fleetManager.fleets()));
        liftStateManager.on('updated', () => setLiftStates(liftStateManager.liftStates()));
        doorStateManager.on('updated', () => setDoorStates(doorStateManager.doorStates()));
        dispenserStateManager.on('updated', () =>
          setDispenserStates(dispenserStateManager.dispenserStates()),
        );
        negotiationStatusManager.on('updated', () =>
          setNegotiationStatus(negotiationStatusManager.allConflicts()),
        );
        setTransport(x);
      })
      .catch((e: CloseEvent) => {
        setLoading({ caption: `Unable to connect to SOSS server (${e.code})`, variant: 'error' });
      });
  }, [
    transportFactory,
    doorStateManager,
    liftStateManager,
    dispenserStateManager,
    fleetManager,
    negotiationStatusManager,
  ]);

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
    setDoors(buildingMap ? buildingMap.levels.flatMap(x => x.doors) : []);
    setLifts(buildingMap ? buildingMap.lifts : []);
  }, [buildingMap]);

  const handleDoorClick = React.useCallback((door: RomiCore.Door) => {
    setShowOmniPanel(true);
    setCurrentView(OmniPanelViewIndex.Doors);
    setDoorSpotlight({ value: door.name });
  }, []);

  const handleRobotClick = React.useCallback((fleet: string, robot: RomiCore.RobotState) => {
    setShowOmniPanel(true);
    setCurrentView(OmniPanelViewIndex.Robots);
    setRobotSpotlight({ value: `${fleet}-${robot.name}` });
  }, []);

  const handleLiftClick = React.useCallback((lift: RomiCore.Lift) => {
    setShowOmniPanel(true);
    setCurrentView(OmniPanelViewIndex.Lifts);
    setLiftSpotlight({ value: lift.name });
  }, []);

  function clearSpotlights() {
    setDoorSpotlight(undefined);
    setLiftSpotlight(undefined);
    setRobotSpotlight(undefined);
    setDispenserSpotlight(undefined);
    setNegotiationSpotlight(undefined);
  }

  const handleClose = React.useCallback(() => {
    clearSpotlights();
    setNegotiationTrajStore({});
    setShowOmniPanel(false);
  }, []);

  const handleBack = React.useCallback(
    (index: number) => {
      clearSpotlights();
      const parent = viewMap[index].parent;
      if (!parent) {
        return handleClose();
      } else {
        setNegotiationTrajStore({});
      }
      setCurrentView(parent.value);
    },
    [handleClose],
  );

  const handleMainMenuDoorsClick = React.useCallback(() => {
    setCurrentView(OmniPanelViewIndex.Doors);
  }, []);

  const handleMainMenuLiftsClick = React.useCallback(() => {
    setCurrentView(OmniPanelViewIndex.Lifts);
  }, []);

  const handleMainMenuRobotsClick = React.useCallback(() => {
    setCurrentView(OmniPanelViewIndex.Robots);
  }, []);

  const handleMainMenuDispensersClick = React.useCallback(() => {
    setCurrentView(OmniPanelViewIndex.Dispensers);
  }, []);

  const handleMainMenuCommandsClick = React.useCallback(() => {
    setCurrentView(OmniPanelViewIndex.Commands);
  }, []);

  const handleMainMenuNegotiationsClick = React.useCallback(() => {
    setCurrentView(OmniPanelViewIndex.Negotiations);
  }, []);

  const omniPanelClasses = React.useMemo(
    () => ({ backButton: classes.topLeftBorder, closeButton: classes.topRightBorder }),
    [classes.topLeftBorder, classes.topRightBorder],
  );

  return (
    <AppContextProvider settings={settings} notificationDispatch={setNotificationBarMessage}>
      <RmfContextProvider doorStates={doorStates} liftStates={liftStates}>
        <div className={classes.container}>
          <AppBar
            toggleShowOmniPanel={() => setShowOmniPanel(!showOmniPanel)}
            showSettings={setShowSettings}
          />
          {loading && <LoadingScreen {...loading} />}
          {buildingMap && (
            <ScheduleVisualizer
              buildingMap={buildingMap}
              fleets={fleets}
              trajManager={trajManager.current}
              appResources={resourceManager.current}
              negotiationTrajStore={negotiationTrajStore}
              onDoorClick={handleDoorClick}
              onLiftClick={handleLiftClick}
              onRobotClick={handleRobotClick}
            />
          )}
          <Fade in={showOmniPanel}>
            <OmniPanel
              className={classes.omniPanel}
              classes={omniPanelClasses}
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
                  onNegotiationsClick={handleMainMenuNegotiationsClick}
                />
              </OmniPanelView>
              <OmniPanelView id={OmniPanelViewIndex.Doors}>
                <DoorsPanel
                  transport={transport}
                  doors={doors}
                  doorStates={doorStates}
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
                <DispensersPanel dispenserStates={dispenserStates} spotlight={dispenserSpotlight} />
              </OmniPanelView>
              <OmniPanelView id={OmniPanelViewIndex.Commands}>
                <CommandsPanel transport={transport} allFleets={fleetNames.current} />
              </OmniPanelView>
              <OmniPanelView id={OmniPanelViewIndex.Negotiations}>
                <NegotiationsPanel
                  conflicts={negotiationStatus}
                  spotlight={negotiationSpotlight}
                  trajManager={trajManager.current}
                  negotiationTrajStore={negotiationTrajStore}
                />
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
      </RmfContextProvider>
    </AppContextProvider>
  );
}
