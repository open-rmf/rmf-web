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
import {
  NegotiationStatusManager,
  NegotiationTrajectoryResponse,
} from '../negotiation-status-manager';
import { ResourceConfigurationsType } from '../resource-manager';
import { RobotTrajectoryManager } from '../robot-trajectory-manager';
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
import SettingsDrawer from './drawers/settings-drawer';
import { SpotlightValue } from './spotlight-value';
import DashboardTour from './tour';
import { buildHotKeys } from '../hotkeys';
import HelpDrawer from './drawers/help-drawer';
import HotKeysDialog from './drawers/hotkeys-dialog';
import { GlobalHotKeys } from 'react-hotkeys';

const debug = Debug('App');
const borderRadius = 20;

const useStyles = makeStyles((theme) => ({
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

export enum OmniPanelViewIndex {
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

  const mapFloorLayerSorted = React.useMemo<string[] | undefined>(
    () => buildingMap?.levels.sort((a, b) => a.elevation - b.elevation).map((x) => x.name),
    [buildingMap],
  );

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
  const newFleetNames = fleets.map((fleet) => fleet.name);
  if (newFleetNames.some((fleetName) => !fleetNames.current.includes(fleetName))) {
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

  const [showHelp, setShowHelp] = React.useState(false);

  const [
    notificationBarMessage,
    setNotificationBarMessage,
  ] = React.useState<NotificationBarProps | null>(null);

  const [tourState, setTourState] = React.useState(false);

  React.useEffect(() => {
    setLoading({ caption: 'Connecting to SOSS server...' });
    transportFactory()
      .then((x) => {
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
      .then((result) => {
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
    setDoors(buildingMap ? buildingMap.levels.flatMap((x) => x.doors) : []);
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

  const setTourSettingsAndOmniPanel = (
    isSettingsVisible: boolean,
    isOmniPanelVisible: boolean,
    clearSpotlight?: boolean,
  ): void => {
    if (clearSpotlight) {
      clearSpotlights();
    }
    setShowSettings(isSettingsVisible);
    setShowOmniPanel(isOmniPanelVisible);
  };

  const setTourShowOmniPanel = (view: OmniPanelViewIndex): void => {
    setTourSettingsAndOmniPanel(false, true, false);
    setCurrentView(view);
  };

  const tourComplete = localStorage.getItem('tourComplete');
  React.useEffect(() => {
    if (tourComplete === 'true') {
      setTourState(false);
    } else {
      setTourState(true);
    }
  }, [tourComplete, setTourState]);

  const tourProps = {
    tourState,
    setTourState,
    setTourSettingsAndOmniPanel,
    setTourShowOmniPanel,
    OmniPanelViewIndex,
    doorSpotlight,
    setDoorSpotlight,
  };

  const [showHotkeyDialog, setShowHotkeyDialog] = React.useState(false);

  const hotKeysValue = buildHotKeys({
    openCommands: () => {
      setShowOmniPanel(true);
      handleMainMenuCommandsClick();
    },
    openRobots: () => {
      setShowOmniPanel(true);
      handleMainMenuRobotsClick();
    },
    openDoors: () => {
      setShowOmniPanel(true);
      handleMainMenuDoorsClick();
    },
    openDispensers: () => {
      setShowOmniPanel(true);
      handleMainMenuDispensersClick();
    },
    openLifts: () => {
      setShowOmniPanel(true);
      handleMainMenuLiftsClick();
    },

    openSettings: () => setShowSettings((prev) => !prev),
    openOnmiPanel: () => setShowOmniPanel((prev) => !prev),
    openHelpPanel: () => setShowHelp((prev) => !prev),
    openHotKeys: () => setShowHotkeyDialog((prev) => !prev),
  });

  return (
    <GlobalHotKeys keyMap={hotKeysValue.keyMap} handlers={hotKeysValue.handlers}>
      <AppContextProvider settings={settings} notificationDispatch={setNotificationBarMessage}>
        <RmfContextProvider doorStates={doorStates} liftStates={liftStates}>
          <div className={classes.container}>
            <AppBar
              toggleShowOmniPanel={() => setShowOmniPanel(!showOmniPanel)}
              showSettings={setShowSettings}
              showHelp={setShowHelp}
              showTour={setTourState}
            />
            {loading && <LoadingScreen {...loading} />}
            {buildingMap && mapFloorLayerSorted && (
              <ScheduleVisualizer
                buildingMap={buildingMap}
                mapFloorLayerSorted={mapFloorLayerSorted}
                fleets={fleets}
                trajManager={trajManager.current}
                negotiationTrajStore={negotiationTrajStore}
                appResources={resourceManager.current}
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
                  <DispensersPanel
                    dispenserStates={dispenserStates}
                    spotlight={dispenserSpotlight}
                  />
                </OmniPanelView>
                <OmniPanelView id={OmniPanelViewIndex.Commands}>
                  <CommandsPanel transport={transport} allFleets={fleetNames.current} />
                </OmniPanelView>
                <OmniPanelView id={OmniPanelViewIndex.Negotiations}>
                  <NegotiationsPanel
                    conflicts={negotiationStatus}
                    spotlight={negotiationSpotlight}
                    mapFloorLayerSorted={mapFloorLayerSorted}
                    negotiationStatusManager={negotiationStatusManager}
                    negotiationTrajStore={negotiationTrajStore}
                  />
                </OmniPanelView>
              </OmniPanel>
            </Fade>

            <SettingsDrawer
              settings={settings}
              open={showSettings}
              onSettingsChange={(newSettings) => {
                setSettings(newSettings);
                saveSettings(newSettings);
              }}
              onClose={() => setShowSettings(false)}
            />

            <HelpDrawer
              open={showHelp}
              handleCloseButton={() => setShowHelp(false)}
              onClose={() => setShowHelp(false)}
              setShowHotkeyDialog={() => setShowHotkeyDialog(true)}
            />
          </div>
          {showHotkeyDialog && (
            <HotKeysDialog open={showHotkeyDialog} handleClose={() => setShowHotkeyDialog(false)} />
          )}
          <NotificationBar
            message={notificationBarMessage?.message}
            type={notificationBarMessage?.type}
          />
          <DashboardTour tourProps={tourProps} />
        </RmfContextProvider>
      </AppContextProvider>
    </GlobalHotKeys>
  );
}
