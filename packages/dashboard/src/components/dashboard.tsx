import { Fade, makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { adapterDoorRequests, adapterLiftRequests, toRosTime } from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import {
  createSpotlightRef,
  defaultDict,
  DispenserAccordion as DispenserAccordion_,
  DoorAccordion as DoorAccordion_,
  LiftAccordion as LiftAccordion_,
  LiftAccordionProps,
  OmniPanel,
  OmniPanelView,
  RobotAccordion as RobotAccordion_,
  StackNavigator,
  withSpotlight,
} from 'react-components';
import { GlobalHotKeys } from 'react-hotkeys';
import 'typeface-roboto';
import appConfig from '../app-config';
import DispenserStateManager from '../dispenser-state-manager';
import DoorStateManager from '../door-state-manager';
import FleetManager from '../fleet-manager';
import { buildHotKeys } from '../hotkeys';
import LiftStateManager from '../lift-state-manager';
import TaskManager from '../managers/task-manager';
import {
  NegotiationStatusManager,
  NegotiationTrajectoryResponse,
} from '../negotiation-status-manager';
import ResourceManager from '../resource-manager';
import { RobotTrajectoryManager } from '../robot-trajectory-manager';
import { loadSettings, saveSettings, Settings } from '../settings';
import { AppContextProvider } from './app-contexts';
import AppBar from './appbar';
import CommandsPanel from './commands-panel';
import HelpDrawer from './drawers/help-drawer';
import HotKeysDialog from './drawers/hotkeys-dialog';
import SettingsDrawer from './drawers/settings-drawer';
import LoadingScreen, { LoadingScreenProps } from './loading-screen';
import MainMenu from './main-menu';
import NegotiationsPanel from './negotiations-panel';
import NotificationBar, { NotificationBarProps } from './notification-bar';
import { RmfContextProvider } from './rmf-contexts';
import ScheduleVisualizer, { ScheduleVisualizerProps } from './schedule-visualizer';
import { SpotlightValue } from './spotlight-value';
import { TaskSummaryPanel } from './task-summary-panel';

const debug = Debug('App');
const DispenserAccordion = React.memo(withSpotlight(DispenserAccordion_));
const DoorAccordion = React.memo(withSpotlight(DoorAccordion_));
const LiftAccordion = React.memo(withSpotlight(LiftAccordion_));
const RobotAccordion = React.memo(withSpotlight(RobotAccordion_));

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
}));

export enum OmniPanelViewIndex {
  MainMenu = 0,
  Doors,
  Lifts,
  Robots,
  Dispensers,
  Commands,
  Negotiations,
  Tasks,
}

function robotKey(fleet: string, robot: RomiCore.RobotState): string {
  return `${fleet}-${robot.name}`;
}

export default function Dashboard(_props: {}): React.ReactElement {
  debug('render');

  const { appResources, transportFactory, trajectoryManagerFactory, trajServerUrl } = appConfig;
  const classes = useStyles();
  const [transport, setTransport] = React.useState<RomiCore.Transport | undefined>(undefined);
  const [buildingMap, setBuildingMap] = React.useState<RomiCore.BuildingMap | undefined>(undefined);
  const trajManager = React.useRef<RobotTrajectoryManager | undefined>(undefined);
  const resourceManager = React.useRef<ResourceManager | undefined>(undefined);

  const mapFloorLayerSorted = React.useMemo<string[] | undefined>(
    () => buildingMap?.levels.sort((a, b) => a.elevation - b.elevation).map((x) => x.name),
    [buildingMap],
  );

  const [showOmniPanel, setShowOmniPanel] = React.useState(true);
  const [currentView, setCurrentView] = React.useState(OmniPanelViewIndex.MainMenu);
  const stackNavigator = React.useMemo(
    () => new StackNavigator<OmniPanelViewIndex>(OmniPanelViewIndex.MainMenu),
    [],
  );

  const pushOmniPanelView = React.useCallback(
    (view: OmniPanelViewIndex) => {
      stackNavigator.push(view);
      setCurrentView(view);
    },
    [stackNavigator],
  );

  const handleOmniPanelClose = React.useCallback(() => {
    clearSpotlights();
    setNegotiationTrajStore({});
    stackNavigator.reset();
    setShowOmniPanel(false);
  }, [stackNavigator]);

  const handleOmniPanelBack = React.useCallback(() => {
    clearSpotlights();
    if (stackNavigator.stack.length === 1) {
      return handleOmniPanelClose();
    } else {
      setNegotiationTrajStore({});
    }
    setCurrentView(stackNavigator.pop());
  }, [stackNavigator, handleOmniPanelClose]);

  const handleOmniPanelHome = React.useCallback(() => {
    clearSpotlights();
    setNegotiationTrajStore({});
    stackNavigator.reset();
    setCurrentView(stackNavigator.top());
  }, [stackNavigator]);

  const doorStateManager = React.useMemo(() => new DoorStateManager(), []);
  const [doorStates, setDoorStates] = React.useState(() => doorStateManager.doorStates());
  const [doors, setDoors] = React.useState<RomiCore.Door[]>([]);
  const doorAccordionRefs = React.useMemo(() => defaultDict(createSpotlightRef), []);
  const handleDoorMarkerClick = React.useCallback(
    (door: RomiCore.Door) => {
      setShowOmniPanel(true);
      pushOmniPanelView(OmniPanelViewIndex.Doors);
      doorAccordionRefs[door.name].spotlight();
    },
    [doorAccordionRefs, pushOmniPanelView],
  );

  const liftStateManager = React.useMemo(() => new LiftStateManager(), []);
  const [liftStates, setLiftStates] = React.useState(() => liftStateManager.liftStates());
  const [lifts, setLifts] = React.useState<RomiCore.Lift[]>([]);
  const liftAccordionRefs = React.useMemo(() => defaultDict(createSpotlightRef), []);
  const handleLiftMarkerClick = React.useCallback(
    (lift: RomiCore.Lift) => {
      setShowOmniPanel(true);
      pushOmniPanelView(OmniPanelViewIndex.Lifts);
      liftAccordionRefs[lift.name].spotlight();
    },
    [liftAccordionRefs, pushOmniPanelView],
  );

  const dispenserStateManager = React.useMemo(() => new DispenserStateManager(), []);
  const [dispenserStates, setDispenserStates] = React.useState<
    Readonly<Record<string, RomiCore.DispenserState>>
  >(() => dispenserStateManager.dispenserStates());
  const dispenserAccordionRefs = React.useMemo(() => defaultDict(createSpotlightRef), []);
  const handleDispenserMarkerClick = React.useCallback<
    Required<ScheduleVisualizerProps>['onDispenserClick']
  >(
    (_, guid) => {
      setShowOmniPanel(true);
      pushOmniPanelView(OmniPanelViewIndex.Dispensers);
      dispenserAccordionRefs[guid].spotlight();
    },
    [dispenserAccordionRefs, pushOmniPanelView],
  );
  const dispensers = React.useMemo(() => {
    if (resourceManager.current?.dispensers) {
      return Object.keys(resourceManager.current?.dispensers.dispensers);
    }
  }, [resourceManager.current]);

  const fleetManager = React.useMemo(() => new FleetManager(), []);
  const [fleets, setFleets] = React.useState(fleetManager.fleets());
  const fleetNames = React.useRef<string[]>([]);
  const newFleetNames = fleets.map((fleet) => fleet.name);
  if (newFleetNames.some((fleetName) => !fleetNames.current.includes(fleetName))) {
    fleetNames.current = newFleetNames;
  }
  const robotAccordionRefs = React.useMemo(() => defaultDict(createSpotlightRef), []);
  const handleRobotMarkerClick = React.useCallback(
    (fleet: string, robot: RomiCore.RobotState) => {
      setShowOmniPanel(true);
      pushOmniPanelView(OmniPanelViewIndex.Robots);
      robotAccordionRefs[robotKey(fleet, robot)].spotlight();
    },
    [robotAccordionRefs, pushOmniPanelView],
  );

  const taskManager = React.useMemo(() => new TaskManager(), []);
  const [tasks, setTasks] = React.useState(taskManager.tasks());

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
  const statusUpdateTS = React.useRef<number>();
  statusUpdateTS.current = negotiationStatusManager.getLastUpdateTS();

  const [loading, setLoading] = React.useState<LoadingScreenProps | null>({
    caption: 'Connecting to api server...',
  });

  const [showSettings, setShowSettings] = React.useState(false);
  const [settings, setSettings] = React.useState<Settings>(() => loadSettings());

  const [showHelp, setShowHelp] = React.useState(false);

  const [
    notificationBarMessage,
    setNotificationBarMessage,
  ] = React.useState<NotificationBarProps | null>(null);

  const [tourState, setTourState] = React.useState(false);
  const [showTooltips, setTooltips] = React.useState(true);

  const toggleTooltips = React.useCallback(() => {
    setTooltips(!showTooltips);
  }, [showTooltips]);

  const tooltipsValue = React.useMemo(() => ({ showTooltips, toggleTooltips }), [
    showTooltips,
    toggleTooltips,
  ]);

  React.useEffect(() => {
    setLoading({ caption: 'Connecting to api server...' });
    transportFactory()
      .then((x) => {
        x.on('error', console.error);
        x.once('close', () => {
          setLoading({ caption: 'Lost connection to api server', variant: 'error' });
          setTransport(undefined);
        });
        doorStateManager.startSubscription(x);
        dispenserStateManager.startSubscription(x);
        liftStateManager.startSubscription(x);
        fleetManager.startSubscription(x);
        negotiationStatusManager.startSubscription();
        taskManager.startSubscription(x);

        fleetManager.on('updated', () => setFleets(fleetManager.fleets()));
        liftStateManager.on('updated', () => setLiftStates(liftStateManager.liftStates()));
        doorStateManager.on('updated', () => setDoorStates(doorStateManager.doorStates()));
        dispenserStateManager.on('updated', () =>
          setDispenserStates(dispenserStateManager.dispenserStates()),
        );
        negotiationStatusManager.on('updated', () =>
          setNegotiationStatus(negotiationStatusManager.allConflicts()),
        );
        taskManager.on('updated', () => setTasks(taskManager.tasks()));
        setTransport(x);
      })
      .catch((e: CloseEvent) => {
        setLoading({ caption: `Unable to connect to api server (${e.code})`, variant: 'error' });
      });
  }, [
    transportFactory,
    doorStateManager,
    liftStateManager,
    dispenserStateManager,
    fleetManager,
    negotiationStatusManager,
    taskManager,
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

  const doorRequestPub = React.useMemo(() => transport?.createPublisher(adapterDoorRequests), [
    transport,
  ]);
  const handleOnDoorControlClick = React.useCallback(
    (_ev, door: RomiCore.Door, mode: number) =>
      doorRequestPub?.publish({
        door_name: door.name,
        request_time: RomiCore.toRosTime(new Date()),
        requested_mode: { value: mode },
        requester_id: 'dashboard',
      }),
    [doorRequestPub],
  );

  const liftRequestPub = React.useMemo(() => transport?.createPublisher(adapterLiftRequests), [
    transport,
  ]);
  const handleLiftRequestSubmit = React.useCallback<
    Required<LiftAccordionProps>['onRequestSubmit']
  >(
    (_ev, lift, doorState, requestType, destination) =>
      liftRequestPub?.publish({
        lift_name: lift.name,
        destination_floor: destination,
        door_state: doorState,
        request_type: requestType,
        request_time: toRosTime(new Date()),
        session_id: 'dashboard',
      }),
    [liftRequestPub],
  );

  function clearSpotlights() {
    setNegotiationSpotlight(undefined);
  }

  const handleMainMenuDoorsClick = React.useCallback(() => {
    pushOmniPanelView(OmniPanelViewIndex.Doors);
  }, [pushOmniPanelView]);

  const handleMainMenuLiftsClick = React.useCallback(() => {
    pushOmniPanelView(OmniPanelViewIndex.Lifts);
  }, [pushOmniPanelView]);

  const handleMainMenuRobotsClick = React.useCallback(() => {
    pushOmniPanelView(OmniPanelViewIndex.Robots);
  }, [pushOmniPanelView]);

  const handleMainMenuDispensersClick = React.useCallback(() => {
    pushOmniPanelView(OmniPanelViewIndex.Dispensers);
  }, [pushOmniPanelView]);

  const handleMainMenuCommandsClick = React.useCallback(() => {
    pushOmniPanelView(OmniPanelViewIndex.Commands);
  }, [pushOmniPanelView]);

  const handleMainMenuNegotiationsClick = React.useCallback(() => {
    pushOmniPanelView(OmniPanelViewIndex.Negotiations);
  }, [pushOmniPanelView]);

  const handleMainMenuTasksClick = React.useCallback(() => {
    pushOmniPanelView(OmniPanelViewIndex.Tasks);
  }, [pushOmniPanelView]);

  const tourComplete = localStorage.getItem('tourComplete');
  React.useEffect(() => {
    if (tourComplete === 'true') {
      setTourState(false);
    } else {
      setTourState(true);
    }
  }, [tourComplete, setTourState]);

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
      <AppContextProvider
        settings={settings}
        notificationDispatch={setNotificationBarMessage}
        resourceManager={resourceManager.current}
        tooltips={tooltipsValue}
      >
        <RmfContextProvider
          doorStates={doorStates}
          liftStates={liftStates}
          dispenserStates={dispenserStates}
        >
          <div className={classes.container}>
            <AppBar
              toggleShowOmniPanel={() => setShowOmniPanel(!showOmniPanel)}
              showSettings={setShowSettings}
              showHelp={setShowHelp}
            />
            {loading && <LoadingScreen {...loading} />}
            {buildingMap && mapFloorLayerSorted && !tourState && (
              <ScheduleVisualizer
                buildingMap={buildingMap}
                mapFloorLayerSorted={mapFloorLayerSorted}
                fleets={fleets}
                trajManager={trajManager.current}
                negotiationTrajStore={negotiationTrajStore}
                onDoorClick={handleDoorMarkerClick}
                onLiftClick={handleLiftMarkerClick}
                onRobotClick={handleRobotMarkerClick}
                onDispenserClick={handleDispenserMarkerClick}
              />
            )}
            <Fade in={showOmniPanel}>
              <OmniPanel
                className={classes.omniPanel}
                view={currentView}
                variant="backHomeClose"
                onBack={handleOmniPanelBack}
                onHome={handleOmniPanelHome}
                onClose={handleOmniPanelClose}
              >
                <OmniPanelView viewId={OmniPanelViewIndex.MainMenu}>
                  <MainMenu
                    onDoorsClick={handleMainMenuDoorsClick}
                    onLiftsClick={handleMainMenuLiftsClick}
                    onRobotsClick={handleMainMenuRobotsClick}
                    onDispensersClick={handleMainMenuDispensersClick}
                    onCommandsClick={handleMainMenuCommandsClick}
                    onNegotiationsClick={handleMainMenuNegotiationsClick}
                    onTasksClick={handleMainMenuTasksClick}
                  />
                </OmniPanelView>
                <OmniPanelView viewId={OmniPanelViewIndex.Doors}>
                  {doors.map((door) => (
                    <DoorAccordion
                      key={door.name}
                      ref={doorAccordionRefs[door.name].ref}
                      door={door}
                      doorState={doorStates[door.name]}
                      onDoorControlClick={handleOnDoorControlClick}
                      data-name={door.name}
                    />
                  ))}
                </OmniPanelView>
                <OmniPanelView viewId={OmniPanelViewIndex.Lifts}>
                  {lifts.map((lift) => (
                    <LiftAccordion
                      key={lift.name}
                      ref={liftAccordionRefs[lift.name].ref}
                      lift={lift}
                      liftState={liftStates[lift.name]}
                      onRequestSubmit={handleLiftRequestSubmit}
                    />
                  ))}
                </OmniPanelView>
                <OmniPanelView viewId={OmniPanelViewIndex.Robots}>
                  {fleets.flatMap((fleet) =>
                    fleet.robots.map((robot) => (
                      <RobotAccordion
                        key={robotKey(fleet.name, robot)}
                        ref={robotAccordionRefs[robotKey(fleet.name, robot)].ref}
                        robot={robot}
                        fleetName={fleet.name}
                        data-component="RobotAccordion"
                      />
                    )),
                  )}
                </OmniPanelView>
                <OmniPanelView viewId={OmniPanelViewIndex.Dispensers}>
                  {Object.values(dispenserStates).map((dispenser) => (
                    <DispenserAccordion
                      key={dispenser.guid}
                      ref={dispenserAccordionRefs[dispenser.guid].ref}
                      dispenserState={dispenser}
                      data-component="DispenserAccordion"
                      dispensers={dispensers}
                    />
                  ))}
                </OmniPanelView>
                <OmniPanelView viewId={OmniPanelViewIndex.Commands}>
                  <CommandsPanel transport={transport} allFleets={fleetNames.current} />
                </OmniPanelView>
                <OmniPanelView viewId={OmniPanelViewIndex.Negotiations}>
                  <NegotiationsPanel
                    conflicts={negotiationStatus}
                    spotlight={negotiationSpotlight}
                    mapFloorLayerSorted={mapFloorLayerSorted}
                    negotiationStatusManager={negotiationStatusManager}
                    negotiationTrajStore={negotiationTrajStore}
                    negotiationStatusUpdateTS={statusUpdateTS.current}
                    setNegotiationTrajStore={setNegotiationTrajStore}
                  />
                </OmniPanelView>
                <OmniPanelView viewId={OmniPanelViewIndex.Tasks}>
                  <TaskSummaryPanel allTasks={tasks} />
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
              handleCloseButton={setShowSettings}
            />

            <HelpDrawer
              open={showHelp}
              handleCloseButton={() => setShowHelp(false)}
              onClose={() => setShowHelp(false)}
              setShowHotkeyDialog={() => setShowHotkeyDialog(true)}
              showTour={() => {
                setTourState(true);
                setShowHelp(false);
              }}
            />
          </div>
          {showHotkeyDialog && (
            <HotKeysDialog open={showHotkeyDialog} handleClose={() => setShowHotkeyDialog(false)} />
          )}

          <NotificationBar
            message={notificationBarMessage?.message}
            type={notificationBarMessage?.type}
          />
        </RmfContextProvider>
      </AppContextProvider>
    </GlobalHotKeys>
  );
}
