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
import { buildHotKeys } from '../hotkeys';
import TaskManager from '../managers/task-manager';
import { NegotiationTrajectoryResponse } from '../negotiation-status-manager';
import ResourceManager from '../resource-manager';
import { RobotTrajectoryManager } from '../robot-trajectory-manager';
import { loadSettings } from '../settings';
import AppBar from './appbar';
import CommandsPanel from './commands-panel';
import { DashboardDrawers } from './dashboard-drawers';
import LoadingScreen, { LoadingScreenProps } from './loading-screen';
import MainMenu from './main-menu';
import NegotiationsPanel from './negotiations-panel';
import { MainMenuState, useMainMenuReducer } from './reducers/main-menu-reducer';
import {
  DepHacksContext,
  DispenserStateContext,
  DoorStateContext,
  FleetStateContext,
  LiftStateContext,
  NegotiationStatusContext,
} from './rmf-contexts';
import ScheduleVisualizer, { ScheduleVisualizerProps } from './schedule-visualizer';
import { SpotlightValue } from './spotlight-value';
import TaskSummaryPanel from './task-summary-panel';

const debug = Debug('App');
const DispenserAccordion = React.memo(withSpotlight(DispenserAccordion_));
const DoorAccordion = React.memo(withSpotlight(DoorAccordion_));
const LiftAccordion = React.memo(withSpotlight(LiftAccordion_));
const RobotAccordion = React.memo(withSpotlight(RobotAccordion_));

const borderRadius = 20;

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

export const mainMenuInitialValues: MainMenuState = {
  currentView: 0,
  loading: {
    caption: 'Connecting to api server...',
  },
  settings: loadSettings(),
  showHelp: false,
  showHotkeysDialog: false,
  showOmniPanel: true,
  showSettings: false,
  stackNavigator: new StackNavigator<OmniPanelViewIndex>(OmniPanelViewIndex.MainMenu),
};

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

function robotKey(fleet: string, robot: RomiCore.RobotState): string {
  return `${fleet}-${robot.name}`;
}

export default function Dashboard(_props: {}): React.ReactElement {
  debug('render');

  const { appResources, transportFactory, trajectoryManagerFactory } = appConfig;
  const classes = useStyles();
  const [transport, setTransport] = React.useState<RomiCore.Transport | undefined>(undefined);
  const [buildingMap, setBuildingMap] = React.useState<RomiCore.BuildingMap | undefined>(undefined);
  const trajManager = React.useRef<RobotTrajectoryManager | undefined>(undefined);
  const resourceManager = React.useRef<ResourceManager | undefined>(undefined);

  const { state: mainMenuState, dispatch: mainMenuDispatch } = useMainMenuReducer(
    mainMenuInitialValues,
  );

  const {
    currentView,
    settings,
    showOmniPanel,
    stackNavigator,
    showSettings,
    showHelp,
    showHotkeysDialog,
  } = mainMenuState;

  const { setCurrentView, setShowOmniPanel, resetView, popView, pushView } = mainMenuDispatch;

  const mapFloorLayerSorted = React.useMemo<string[] | undefined>(
    () => buildingMap?.levels.sort((a, b) => a.elevation - b.elevation).map((x) => x.name),
    [buildingMap],
  );

  const handleOmniPanelClose = React.useCallback(() => {
    clearSpotlights();
    setNegotiationTrajStore({});
    resetView();
    setShowOmniPanel(false);
  }, [resetView, setShowOmniPanel]);

  const handleOmniPanelBack = React.useCallback(() => {
    clearSpotlights();
    if (stackNavigator.stack.length === 1) {
      return handleOmniPanelClose();
    } else {
      setNegotiationTrajStore({});
    }
    popView();
  }, [stackNavigator, handleOmniPanelClose, popView]);

  const handleOmniPanelHome = React.useCallback(() => {
    clearSpotlights();
    setNegotiationTrajStore({});
    resetView();
    setCurrentView(stackNavigator.top());
  }, [resetView, setCurrentView, stackNavigator]);

  const doorStates = React.useContext(DoorStateContext);
  const [doors, setDoors] = React.useState<RomiCore.Door[]>([]);
  const doorAccordionRefs = React.useMemo(() => defaultDict(createSpotlightRef), []);
  const handleDoorMarkerClick = React.useCallback(
    (door: RomiCore.Door) => {
      setShowOmniPanel(true);
      pushView(OmniPanelViewIndex.Doors);
      doorAccordionRefs[door.name].spotlight();
    },
    [doorAccordionRefs, pushView, setShowOmniPanel],
  );

  const liftStates = React.useContext(LiftStateContext);
  const [lifts, setLifts] = React.useState<RomiCore.Lift[]>([]);
  const liftAccordionRefs = React.useMemo(() => defaultDict(createSpotlightRef), []);
  const handleLiftMarkerClick = React.useCallback(
    (lift: RomiCore.Lift) => {
      setShowOmniPanel(true);
      pushView(OmniPanelViewIndex.Lifts);
      liftAccordionRefs[lift.name].spotlight();
    },
    [liftAccordionRefs, pushView, setShowOmniPanel],
  );

  const dispenserStates = React.useContext(DispenserStateContext);
  const dispenserAccordionRefs = React.useMemo(() => defaultDict(createSpotlightRef), []);
  const handleDispenserMarkerClick = React.useCallback<
    Required<ScheduleVisualizerProps>['onDispenserClick']
  >(
    (_, guid) => {
      setShowOmniPanel(true);
      pushView(OmniPanelViewIndex.Dispensers);
      dispenserAccordionRefs[guid].spotlight();
    },
    [dispenserAccordionRefs, pushView, setShowOmniPanel],
  );
  let dispensers: string[] | undefined;
  if (resourceManager.current && resourceManager.current.dispensers) {
    dispensers = Object.keys(resourceManager.current.dispensers.dispensers);
  }

  const fleetStates = React.useContext(FleetStateContext);
  const fleets = React.useMemo(() => Object.values(fleetStates), [fleetStates]);
  const fleetNames = React.useRef<string[]>([]);
  const newFleetNames = Object.keys(fleetStates);
  if (newFleetNames.some((fleetName) => !fleetNames.current.includes(fleetName))) {
    fleetNames.current = newFleetNames;
  }
  const robotAccordionRefs = React.useMemo(() => defaultDict(createSpotlightRef), []);
  const handleRobotMarkerClick = React.useCallback(
    (fleet: string, robot: RomiCore.RobotState) => {
      setShowOmniPanel(true);
      pushView(OmniPanelViewIndex.Robots);
      robotAccordionRefs[robotKey(fleet, robot)].spotlight();
    },
    [robotAccordionRefs, pushView, setShowOmniPanel],
  );

  const taskManager = React.useMemo(() => new TaskManager(), []);
  const [tasks, setTasks] = React.useState(taskManager.tasks());

  const negotiationStatus = React.useContext(NegotiationStatusContext);
  const { negotiationStatusManager } = React.useContext(DepHacksContext);
  const [negotiationSpotlight, setNegotiationSpotlight] = React.useState<
    SpotlightValue<string> | undefined
  >(undefined);
  const [negotiationTrajStore, setNegotiationTrajStore] = React.useState<
    Record<string, NegotiationTrajectoryResponse>
  >({});
  const statusUpdateTS = React.useRef<number>();
  statusUpdateTS.current = negotiationStatusManager.getLastUpdateTS();

  const [loading, setLoading] = React.useState<LoadingScreenProps | null>({
    caption: 'Connecting to server...',
  });

  React.useEffect(() => {
    setLoading({ caption: 'Connecting to server...' });
    transportFactory()
      .then((x) => {
        x.on('error', console.error);
        x.once('close', () => {
          setLoading({ caption: 'Lost connection to server', variant: 'error' });
          setTransport(undefined);
        });
        setTransport(x);
      })
      .catch((e: CloseEvent) => {
        setLoading({ caption: `Unable to connect to server (${e.code})`, variant: 'error' });
      });
  }, [transportFactory, taskManager]);

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

  const hotKeysValue = React.useMemo(
    () => buildHotKeys({ reducerMainMenuDispatch: mainMenuDispatch }),
    [mainMenuDispatch],
  );

  return (
    <GlobalHotKeys keyMap={hotKeysValue.keyMap} handlers={hotKeysValue.handlers}>
      <div className={classes.container}>
        <AppBar reducerMainMenuDispatch={mainMenuDispatch} />
        {loading && <LoadingScreen {...loading} />}
        {buildingMap && mapFloorLayerSorted && (
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
              <MainMenu pushView={pushView} />
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
              {dispensers
                ? dispensers.map((dispenser) => (
                    <DispenserAccordion
                      key={dispenser}
                      ref={
                        dispenserStates[dispenser]
                          ? dispenserAccordionRefs[dispenserStates[dispenser].guid].ref
                          : null
                      }
                      dispenserState={
                        dispenserStates[dispenser] ? dispenserStates[dispenser] : null
                      }
                      data-component="DispenserAccordion"
                      dispenser={dispenser}
                    />
                  ))
                : null}
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
              <TaskSummaryPanel tasks={tasks} />
            </OmniPanelView>
          </OmniPanel>
        </Fade>
        <DashboardDrawers
          settings={settings}
          showSettings={showSettings}
          showHelp={showHelp}
          showHotkeysDialog={showHotkeysDialog}
          reducerMainMenuDispatch={mainMenuDispatch}
        ></DashboardDrawers>
      </div>
    </GlobalHotKeys>
  );
}
