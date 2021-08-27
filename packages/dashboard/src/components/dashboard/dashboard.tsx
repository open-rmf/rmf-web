import { Fleet, Level } from 'api-client';
import Debug from 'debug';
import React from 'react';
import { createSpotlightRef, defaultDict, useStackNavigator } from 'react-components';
import { GlobalHotKeys } from 'react-hotkeys';
import * as RmfModels from 'rmf-models';
import { buildHotKeys } from '../../hotkeys';
import {
  NegotiationConflict,
  NegotiationTrajectoryResponse,
} from '../../managers/negotiation-status-manager';
import { AppConfigContext, AppControllerContext } from '../app-contexts';
import {
  BuildingMapContext,
  DispensersContext,
  IngestorsContext,
  RmfIngressContext,
} from '../rmf-app';
import ScheduleVisualizer, { ScheduleVisualizerProps } from '../schedule-visualizer';
import { RobotsOverlayProps } from '../schedule-visualizer/robots-overlay';
import OmniPanelControl_ from './omnipanel-control';
import { DashboardState, useDashboardReducer } from './reducers/dashboard-reducer';

const debug = Debug('Dashboard');
const OmniPanelControl = React.memo(OmniPanelControl_);

export enum OmniPanelViewIndex {
  MainMenu = 0,
  Doors,
  Lifts,
  Robots,
  Dispensers,
  Negotiations,
}

export const dashboardInitialValues: DashboardState = {
  showOmniPanel: true,
};

function robotKey(fleet: string, robotName: string): string {
  return `${fleet}-${robotName}`;
}

const initialStack = [OmniPanelViewIndex.MainMenu];

export default function Dashboard(_props: {}): React.ReactElement {
  debug('render');

  const appConfig = React.useContext(AppConfigContext);
  const appController = React.useContext(AppControllerContext);
  const rmfIngress = React.useContext(RmfIngressContext);
  const sioClient = rmfIngress?.sioClient;

  const buildingMap = React.useContext(BuildingMapContext);

  const { state: dashboardState, dispatch: dashboardDispatch } = useDashboardReducer(
    dashboardInitialValues,
  );

  const [viewStack, viewStackDispatch] = useStackNavigator<OmniPanelViewIndex>(
    initialStack,
    OmniPanelViewIndex.MainMenu,
  );
  const currentView = viewStack[viewStack.length - 1];

  const { showOmniPanel } = dashboardState;
  const { setShowOmniPanel } = dashboardDispatch;

  const mapFloorSort = React.useCallback(
    (levels: RmfModels.Level[]) =>
      levels.sort((a, b) => a.elevation - b.elevation).map((x) => x.name),
    [],
  );

  const [doorStates, setDoorStates] = React.useState<Record<string, RmfModels.DoorState>>({});
  const doors: RmfModels.Door[] = React.useMemo(
    () => (buildingMap ? (buildingMap.levels as Level[]).flatMap((x) => x.doors) : []),
    [buildingMap],
  );
  React.useEffect(() => {
    if (!sioClient) return;
    const subs = doors.map((d) =>
      sioClient.subscribeDoorState(d.name, (state) =>
        setDoorStates((prev) => ({ ...prev, [d.name]: state })),
      ),
    );
    return () => {
      subs.forEach((s) => sioClient.unsubscribe(s));
    };
  }, [sioClient, doors]);
  const doorAccordionRefs = React.useMemo(() => defaultDict(createSpotlightRef), []);
  const handleDoorMarkerClick = React.useCallback(
    (door: RmfModels.Door) => {
      setShowOmniPanel(true);
      viewStackDispatch.push(OmniPanelViewIndex.Doors);
      doorAccordionRefs[door.name].spotlight();
      setFilter('');
    },
    [doorAccordionRefs, viewStackDispatch, setShowOmniPanel],
  );

  const [liftStates, setLiftStates] = React.useState<Record<string, RmfModels.LiftState>>({});
  const lifts: RmfModels.Lift[] = React.useMemo(() => (buildingMap ? buildingMap.lifts : []), [
    buildingMap,
  ]);
  React.useEffect(() => {
    if (!sioClient) return;
    const subs = lifts.map((l) =>
      sioClient.subscribeLiftState(l.name, (state) =>
        setLiftStates((prev) => ({ ...prev, [l.name]: state })),
      ),
    );
    return () => {
      subs.forEach((s) => sioClient.unsubscribe(s));
    };
  }, [sioClient, lifts]);
  const liftAccordionRefs = React.useMemo(() => defaultDict(createSpotlightRef), []);
  const handleLiftMarkerClick = React.useCallback(
    (lift: RmfModels.Lift) => {
      setShowOmniPanel(true);
      viewStackDispatch.push(OmniPanelViewIndex.Lifts);
      liftAccordionRefs[lift.name].spotlight();
      setFilter('');
    },
    [liftAccordionRefs, viewStackDispatch, setShowOmniPanel],
  );

  const dispensers = React.useContext(DispensersContext);
  const [dispenserStates, setDispenserStates] = React.useState<
    Record<string, RmfModels.DispenserState>
  >({});
  React.useEffect(() => {
    if (!sioClient) return;
    const subs = dispensers.map((d) =>
      sioClient.subscribeDispenserState(d.guid, (state) =>
        setDispenserStates((prev) => ({ ...prev, [d.guid]: state })),
      ),
    );
    return () => {
      subs.forEach((s) => sioClient.unsubscribe(s));
    };
  }, [sioClient, dispensers]);

  const ingestors = React.useContext(IngestorsContext);
  const [ingestorStates, setIngestorStates] = React.useState<
    Record<string, RmfModels.IngestorState>
  >({});
  React.useEffect(() => {
    if (!sioClient) return;
    const subs = ingestors.map((d) =>
      sioClient.subscribeIngestorState(d.guid, (state) =>
        setIngestorStates((prev) => ({ ...prev, [d.guid]: state })),
      ),
    );
    return () => {
      subs.forEach((s) => sioClient.unsubscribe(s));
    };
  }, [sioClient, ingestors]);

  const workcellAccordionRefs = React.useMemo(() => defaultDict(createSpotlightRef), []);
  const handleWorkcellMarkerClick = React.useCallback<
    Required<ScheduleVisualizerProps>['onDispenserClick']
  >(
    (_, guid) => {
      setShowOmniPanel(true);
      viewStackDispatch.push(OmniPanelViewIndex.Dispensers);
      workcellAccordionRefs[guid].spotlight();
      setFilter('');
    },
    [workcellAccordionRefs, viewStackDispatch, setShowOmniPanel],
  );

  const [fleets, setFleets] = React.useState<Fleet[]>([]);
  React.useEffect(() => {
    if (!rmfIngress) return;
    let cancel = false;
    (async () => {
      const result = await rmfIngress.fleetsApi.getFleetsFleetsGet();
      if (cancel || result.status !== 200) return;
      setFleets(result.data);
    })();
    return () => {
      cancel = true;
    };
  }, [rmfIngress]);
  const [fleetStates, setFleetStates] = React.useState<Record<string, RmfModels.FleetState>>({});
  React.useEffect(() => {
    if (!sioClient) return;
    const subs = fleets.map((f) =>
      sioClient.subscribeFleetState(f.name, (state) =>
        setFleetStates((prev) => ({ ...prev, [f.name]: state })),
      ),
    );
    return () => {
      subs.forEach((s) => sioClient.unsubscribe(s));
    };
  }, [sioClient, fleets]);
  const fleetNames = React.useRef<string[]>([]);
  const newFleetNames = Object.keys(fleetStates);
  if (newFleetNames.some((fleetName) => !fleetNames.current.includes(fleetName))) {
    fleetNames.current = newFleetNames;
  }
  const robotAccordionRefs = React.useMemo(() => defaultDict(createSpotlightRef), []);
  const handleRobotMarkerClick = React.useCallback<Required<RobotsOverlayProps>['onRobotClick']>(
    (_ev, fleet, robot) => {
      setShowOmniPanel(true);
      viewStackDispatch.push(OmniPanelViewIndex.Robots);
      robotAccordionRefs[robotKey(fleet, robot)].spotlight();
      setFilter('');
    },
    [robotAccordionRefs, viewStackDispatch, setShowOmniPanel],
  );

  const { negotiationStatusManager } = React.useContext(RmfIngressContext) || {};
  const [negotiationStatus, setNegotiationStatus] = React.useState<
    Record<number, NegotiationConflict>
  >({});
  React.useEffect(() => {
    if (!negotiationStatusManager) {
      return;
    }
    negotiationStatusManager.startSubscription(appConfig.authenticator.token);
    const onUpdated = () => setNegotiationStatus(negotiationStatusManager.allConflicts());
    negotiationStatusManager.on('updated', onUpdated);
    // FIXME: unable to unsubscribe

    return () => {
      negotiationStatusManager.off('updated', onUpdated);
    };
  }, [negotiationStatusManager, appConfig]);
  const [negotiationTrajStore, setNegotiationTrajStore] = React.useState<
    Record<string, NegotiationTrajectoryResponse>
  >({});
  const statusUpdateTS = React.useRef<number>();
  statusUpdateTS.current = negotiationStatusManager?.getLastUpdateTS() || -1;

  const hotKeysValue = React.useMemo(
    () =>
      buildHotKeys({
        reducerDashboardDispatch: dashboardDispatch,
        viewStackDispatch,
        appController,
      }),
    [dashboardDispatch, viewStackDispatch, appController],
  );

  const [filter, setFilter] = React.useState('');

  return (
    <GlobalHotKeys keyMap={hotKeysValue.keyMap} handlers={hotKeysValue.handlers}>
      {buildingMap && (
        <>
          <ScheduleVisualizer
            buildingMap={buildingMap}
            doorStates={doorStates}
            liftStates={liftStates}
            fleetStates={fleetStates}
            mapFloorSort={mapFloorSort}
            negotiationTrajStore={negotiationTrajStore}
            showTrajectories={!(currentView === OmniPanelViewIndex.Negotiations)}
            onDoorClick={handleDoorMarkerClick}
            onLiftClick={handleLiftMarkerClick}
            onRobotClick={handleRobotMarkerClick}
            onDispenserClick={handleWorkcellMarkerClick}
          >
            <OmniPanelControl show={!showOmniPanel} dashboardDispatch={dashboardDispatch} />
          </ScheduleVisualizer>
        </>
      )}
    </GlobalHotKeys>
  );
}
