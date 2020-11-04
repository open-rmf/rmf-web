import { Fade, makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { adapterDoorRequests } from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import {
  createSpotlightRef,
  DoorAccordion as DoorAccordion_,
  SpotlightRef,
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
import {
  NegotiationStatusManager,
  NegotiationTrajectoryResponse,
} from '../negotiation-status-manager';
import ResourceManager from '../resource-manager';
import { RobotTrajectoryManager } from '../robot-trajectory-manager';
import { loadSettings, saveSettings } from '../settings';
import { AppContextProvider } from './app-contexts';
import AppBar from './appbar';
import CommandsPanel from './commands-panel';
import DispensersPanel from './dispensers-panel';
import HelpDrawer from './drawers/help-drawer';
import HotKeysDialog from './drawers/hotkeys-dialog';
import SettingsDrawer from './drawers/settings-drawer';
import LiftsPanel from './lift-item/lifts-panel';
import LoadingScreen from './loading-screen';
import MainMenu from './main-menu';
import NegotiationsPanel from './negotiations-panel';
import NotificationBar, { NotificationBarProps } from './notification-bar';
import OmniPanel from './omni-panel';
import OmniPanelView from './omni-panel-view';
import {
  DashboardActionType,
  dashboardReducer,
  DashboardState,
} from './reducers/dashboard-reducers';
import { MainMenuActionType, mainMenuReducer, MainMenuState } from './reducers/main-menu-reducer';
import { RmfContextProvider } from './rmf-contexts';
import RobotsPanel from './robots-panel';
import ScheduleVisualizer from './schedule-visualizer';
import { SpotlightValue } from './spotlight-value';
import { DashboardTour, DashboardTourProps } from './tour/tour';

const debug = Debug('App');
const DoorAccordion = withSpotlight(DoorAccordion_);

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
  const resourceManager = React.useRef<ResourceManager | undefined>(undefined);

  const mapFloorLayerSorted = React.useMemo<string[] | undefined>(
    () => buildingMap?.levels.sort((a, b) => a.elevation - b.elevation).map((x) => x.name),
    [buildingMap],
  );

  const doorStateManager = React.useMemo(() => new DoorStateManager(), []);
  const liftStateManager = React.useMemo(() => new LiftStateManager(), []);

  const initialValues: DashboardState = {
    doorStates: doorStateManager.doorStates(),
    doors: [],
    liftStates: liftStateManager.liftStates(),
    lifts: [],
  };
  const [state, dispatch] = React.useReducer(dashboardReducer, initialValues);

  const mainMenuInitialValues: MainMenuState = {
    showOmniPanel: true,
    currentView: OmniPanelViewIndex.MainMenu,
    loading: {
      caption: 'Connecting to api server...',
    },
    showSettings: false,
    settings: loadSettings(),
    showHelp: false,
    tourState: false,
  };
  const [stateMenu, dispatchMenu] = React.useReducer(mainMenuReducer, mainMenuInitialValues);

  const doorAccordionRefs = React.useMemo(
    () =>
      state.doors.reduce<Record<string, SpotlightRef>>((prev, door) => {
        prev[door.name] = createSpotlightRef();
        return prev;
      }, {}),
    [state.doors],
  );
  const handleDoorMarkerClick = React.useCallback(
    (door: RomiCore.Door) => {
      dispatchMenu({ type: MainMenuActionType.SHOW_OMNIPANEL, payload: true });
      dispatchMenu({ type: MainMenuActionType.CURRENT_VIEW, payload: OmniPanelViewIndex.Doors });
      doorAccordionRefs[door.name].spotlight();
    },
    [doorAccordionRefs],
  );

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
  >(() => dispenserStateManager.dispenserStates());
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
  const statusUpdateTS = React.useRef<number>();
  statusUpdateTS.current = negotiationStatusManager.getLastUpdateTS();

  // const [showOmniPanel, setShowOmniPanel] = React.useState(true);
  // const [currentView, setCurrentView] = React.useState(OmniPanelViewIndex.MainMenu);
  // const [loading, setLoading] = React.useState<LoadingScreenProps | null>({
  //   caption: 'Connecting to api server...',
  // });

  // const [showSettings, setShowSettings] = React.useState(false);
  // const [settings, setSettings] = React.useState<Settings>(() => loadSettings());

  // const [showHelp, setShowHelp] = React.useState(false);

  const [
    notificationBarMessage,
    setNotificationBarMessage,
  ] = React.useState<NotificationBarProps | null>(null);

  // const [tourState, setTourState] = React.useState(false);

  React.useEffect(() => {
    dispatchMenu({
      type: MainMenuActionType.LOADING,
      payload: { caption: 'Connecting to api server...' },
    });
    transportFactory()
      .then((x) => {
        x.on('error', console.error);
        x.once('close', () => {
          dispatchMenu({
            type: MainMenuActionType.LOADING,
            payload: { caption: 'Lost connection to api server', variant: 'error' },
          });
          setTransport(undefined);
        });
        doorStateManager.startSubscription(x);
        dispenserStateManager.startSubscription(x);
        liftStateManager.startSubscription(x);
        fleetManager.startSubscription(x);
        negotiationStatusManager.startSubscription();

        fleetManager.on('updated', () => setFleets(fleetManager.fleets()));
        liftStateManager.on('updated', () =>
          dispatch({
            type: DashboardActionType.LIFT_STATE,
            payload: liftStateManager.liftStates(),
          }),
        );
        doorStateManager.on('updated', () =>
          dispatch({
            type: DashboardActionType.DOOR_STATE,
            payload: doorStateManager.doorStates(),
          }),
        );
        dispenserStateManager.on('updated', () =>
          setDispenserStates(dispenserStateManager.dispenserStates()),
        );
        negotiationStatusManager.on('updated', () =>
          setNegotiationStatus(negotiationStatusManager.allConflicts()),
        );
        setTransport(x);
      })
      .catch((e: CloseEvent) => {
        dispatchMenu({
          type: MainMenuActionType.LOADING,
          payload: { caption: `Unable to connect to api server (${e.code})`, variant: 'error' },
        });
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
    dispatchMenu({
      type: MainMenuActionType.LOADING,
      payload: { caption: 'Downloading building map...' },
    });
    const request = new RomiCore.GetBuildingMap_Request();
    transport
      .call(RomiCore.getBuildingMap, request)
      .then((result) => {
        setBuildingMap(result.building_map);
        dispatchMenu({
          type: MainMenuActionType.LOADING,
          payload: null,
        });
      })
      .catch(() => {
        dispatchMenu({
          type: MainMenuActionType.LOADING,
          payload: { caption: 'Unable to download building map', variant: 'error' },
        });
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
    dispatch({
      type: 'doors',
      payload: buildingMap ? buildingMap.levels.flatMap((x) => x.doors) : [],
    });
    dispatch({
      type: 'lifts',
      payload: buildingMap ? buildingMap.lifts : [],
    });
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

  const handleRobotClick = React.useCallback((fleet: string, robot: RomiCore.RobotState) => {
    dispatchMenu({ type: MainMenuActionType.SHOW_OMNIPANEL, payload: true });
    dispatchMenu({
      type: MainMenuActionType.CURRENT_VIEW,
      payload: OmniPanelViewIndex.Robots,
    });
    setRobotSpotlight({ value: `${fleet}-${robot.name}` });
  }, []);

  const handleLiftClick = React.useCallback((lift: RomiCore.Lift) => {
    dispatchMenu({ type: MainMenuActionType.SHOW_OMNIPANEL, payload: true });
    dispatchMenu({
      type: MainMenuActionType.CURRENT_VIEW,
      payload: OmniPanelViewIndex.Lifts,
    });
    setLiftSpotlight({ value: lift.name });
  }, []);

  function handleDispenserClick(dispenser: RomiCore.DispenserState): void {
    dispatchMenu({ type: MainMenuActionType.SHOW_OMNIPANEL, payload: true });
    dispatchMenu({
      type: MainMenuActionType.CURRENT_VIEW,
      payload: OmniPanelViewIndex.Dispensers,
    });
    setDispenserSpotlight({ value: dispenser.guid });
  }

  function clearSpotlights() {
    setLiftSpotlight(undefined);
    setRobotSpotlight(undefined);
    setDispenserSpotlight(undefined);
    setNegotiationSpotlight(undefined);
  }

  const handleClose = React.useCallback(() => {
    clearSpotlights();
    setNegotiationTrajStore({});
    dispatchMenu({ type: MainMenuActionType.SHOW_OMNIPANEL, payload: false });
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
      dispatchMenu({
        type: MainMenuActionType.CURRENT_VIEW,
        payload: parent.value,
      });
    },
    [handleClose],
  );

  const omniPanelClasses = React.useMemo(
    () => ({ backButton: classes.topLeftBorder, closeButton: classes.topRightBorder }),
    [classes.topLeftBorder, classes.topRightBorder],
  );

  const tourComplete = localStorage.getItem('tourComplete');
  React.useEffect(() => {
    if (tourComplete === 'true') {
      dispatchMenu({ type: MainMenuActionType.TOUR_STATE, payload: false });
    } else {
      dispatchMenu({ type: MainMenuActionType.TOUR_STATE, payload: true });
    }
  }, [tourComplete]);

  const tourProps = React.useMemo<DashboardTourProps['tourProps']>(() => {
    const doorAccordionRef = Object.values(doorAccordionRefs)[0] as SpotlightRef | undefined;
    return {
      dispatchMenu,
      stateMenu,
      clearSpotlights,
      doorSpotlight: doorAccordionRef?.spotlight,
    };
  }, [doorAccordionRefs, stateMenu.tourState]);

  const [showHotkeyDialog, setShowHotkeyDialog] = React.useState(false);

  const hotKeysValue = buildHotKeys({ menuStateHandler: dispatchMenu });

  return (
    <GlobalHotKeys keyMap={hotKeysValue.keyMap} handlers={hotKeysValue.handlers}>
      <AppContextProvider
        settings={stateMenu.settings}
        notificationDispatch={setNotificationBarMessage}
        resourceManager={resourceManager.current}
      >
        <RmfContextProvider
          doorStates={state.doorStates}
          liftStates={state.liftStates}
          dispenserStates={dispenserStates}
        >
          <div className={classes.container}>
            <AppBar mainMenuStateHandler={dispatchMenu} />
            {stateMenu.loading && <LoadingScreen {...stateMenu.loading} />}
            {buildingMap && mapFloorLayerSorted && (
              <ScheduleVisualizer
                buildingMap={buildingMap}
                mapFloorLayerSorted={mapFloorLayerSorted}
                fleets={fleets}
                trajManager={trajManager.current}
                negotiationTrajStore={negotiationTrajStore}
                onDoorClick={handleDoorMarkerClick}
                onLiftClick={handleLiftClick}
                onRobotClick={handleRobotClick}
                onDispenserClick={handleDispenserClick}
              />
            )}

            <Fade in={stateMenu.showOmniPanel}>
              <OmniPanel
                className={classes.omniPanel}
                classes={omniPanelClasses}
                view={stateMenu.currentView}
                onBack={handleBack}
                onClose={handleClose}
              >
                <OmniPanelView id={OmniPanelViewIndex.MainMenu}>
                  <MainMenu mainMenuStateHandler={dispatchMenu} />
                </OmniPanelView>
                <OmniPanelView id={OmniPanelViewIndex.Doors}>
                  {state.doors.map((door) => (
                    <DoorAccordion
                      key={door.name}
                      ref={doorAccordionRefs[door.name].ref}
                      door={door}
                      doorState={state.doorStates[door.name]}
                      onDoorControlClick={handleOnDoorControlClick}
                      data-name={door.name}
                    />
                  ))}
                </OmniPanelView>
                <OmniPanelView id={OmniPanelViewIndex.Lifts}>
                  <LiftsPanel
                    transport={transport}
                    liftStates={state.liftStates}
                    lifts={state.lifts}
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
                    negotiationStatusUpdateTS={statusUpdateTS.current}
                    setNegotiationTrajStore={setNegotiationTrajStore}
                  />
                </OmniPanelView>
              </OmniPanel>
            </Fade>

            <SettingsDrawer
              settings={stateMenu.settings}
              open={stateMenu.showSettings}
              onSettingsChange={(newSettings) => {
                dispatchMenu({ type: MainMenuActionType.SETTINGS, payload: newSettings });
                saveSettings(newSettings);
              }}
              onClose={() =>
                dispatchMenu({ type: MainMenuActionType.SHOW_SETTINGS, payload: false })
              }
              handleCloseButton={() =>
                dispatchMenu({ type: MainMenuActionType.SHOW_SETTINGS, payload: false })
              }
            />

            <HelpDrawer
              open={stateMenu.showHelp}
              handleCloseButton={() =>
                dispatchMenu({ type: MainMenuActionType.SHOW_HELP, payload: false })
              }
              onClose={() => dispatchMenu({ type: MainMenuActionType.SHOW_HELP, payload: false })}
              setShowHotkeyDialog={() => setShowHotkeyDialog(true)}
              showTour={() => {
                dispatchMenu({ type: MainMenuActionType.TOUR_STATE, payload: true });
                dispatchMenu({ type: MainMenuActionType.SHOW_HELP, payload: false });
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
          <DashboardTour tourProps={tourProps} />
        </RmfContextProvider>
      </AppContextProvider>
    </GlobalHotKeys>
  );
}
