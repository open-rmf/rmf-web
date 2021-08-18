import { makeStyles } from '@material-ui/core';
import { Fleet, Level } from 'api-client';
import Debug from 'debug';
import React from 'react';
import {
  createSpotlightRef,
  defaultDict,
  DoorPanel,
  DoorData,
  LiftPanel,
  LiftPanelProps,
  DispenserPanel,
} from 'react-components';
import { GlobalHotKeys } from 'react-hotkeys';
import * as RmfModels from 'rmf-models';
import { buildHotKeys } from '../../hotkeys';
import { AppControllerContext } from '../app-contexts';
import {
  BuildingMapContext,
  DispensersContext,
  IngestorsContext,
  RmfIngressContext,
} from '../rmf-app';
import ScheduleVisualizer, { ScheduleVisualizerProps } from '../schedule-visualizer';
import { RobotsOverlayProps } from '../schedule-visualizer/robots-overlay';

const debug = Debug('Dashboard');

const useStyles = makeStyles(() => ({
  toolBarTitle: {
    flexGrow: 1,
  },
  buildingPanel: {
    display: 'flex',
    height: '100%',
  },
  itemPanels: {
    width: '1100px',
  },
}));

export enum OmniPanelViewIndex {
  MainMenu = 0,
  Doors,
  Lifts,
  Robots,
  Dispensers,
  Negotiations,
}

function robotKey(fleet: string, robotName: string): string {
  return `${fleet}-${robotName}`;
}

export default function Dashboard(_props: {}): React.ReactElement {
  debug('render');

  const classes = useStyles();
  const appController = React.useContext(AppControllerContext);
  const rmfIngress = React.useContext(RmfIngressContext);
  const sioClient = rmfIngress?.sioClient;
  const buildingMap = React.useContext(BuildingMapContext);

  const mapFloorSort = React.useCallback(
    (levels: RmfModels.Level[]) =>
      levels.sort((a, b) => a.elevation - b.elevation).map((x) => x.name),
    [],
  );

  const [doorStates, setDoorStates] = React.useState<Record<string, RmfModels.DoorState>>({});
  const doors: DoorData[] = React.useMemo(() => {
    return buildingMap
      ? (buildingMap.levels as Level[]).flatMap((x) =>
          (x.doors as RmfModels.Door[]).map((door) => ({ door, level: x.name })),
        )
      : [];
  }, [buildingMap]);

  React.useEffect(() => {
    if (!sioClient) return;
    const subs = doors.map((d) =>
      sioClient.subscribeDoorState(d.door.name, (state) =>
        setDoorStates((prev) => ({ ...prev, [d.door.name]: state })),
      ),
    );
    return () => {
      subs.forEach((s) => sioClient.unsubscribe(s));
    };
  }, [sioClient, doors]);
  const doorAccordionRefs = React.useMemo(() => defaultDict(createSpotlightRef), []);
  const handleDoorMarkerClick = React.useCallback(
    (door: RmfModels.Door) => {
      doorAccordionRefs[door.name].spotlight();
    },
    [doorAccordionRefs],
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
      liftAccordionRefs[lift.name].spotlight();
    },
    [liftAccordionRefs],
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

  const workcells = React.useMemo(() => [...dispensers, ...ingestors], [dispensers, ingestors]);
  const workcellStates = React.useMemo(() => ({ ...dispenserStates, ...ingestorStates }), [
    dispenserStates,
    ingestorStates,
  ]);
  const workcellAccordionRefs = React.useMemo(() => defaultDict(createSpotlightRef), []);
  const handleWorkcellMarkerClick = React.useCallback<
    Required<ScheduleVisualizerProps>['onDispenserClick']
  >(
    (_, guid) => {
      workcellAccordionRefs[guid].spotlight();
    },
    [workcellAccordionRefs],
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
      robotAccordionRefs[robotKey(fleet, robot)].spotlight();
    },
    [robotAccordionRefs],
  );

  const { doorsApi, liftsApi } = React.useContext(RmfIngressContext) || {};

  const handleOnDoorControlClick = React.useCallback(
    (_ev, door: RmfModels.Door, mode: number) =>
      doorsApi?.postDoorRequestDoorsDoorNameRequestPost(
        {
          mode: mode,
        },
        door.name,
      ),
    [doorsApi],
  );

  const handleLiftRequestSubmit = React.useCallback<Required<LiftPanelProps>['onRequestSubmit']>(
    (_ev, lift, doorState, requestType, destination) =>
      liftsApi?.postLiftRequestLiftsLiftNameRequestPost(
        {
          destination,
          request_type: requestType,
          door_mode: doorState,
        },
        lift.name,
      ),
    [liftsApi],
  );

  const hotKeysValue = React.useMemo(
    () =>
      buildHotKeys({
        appController,
      }),
    [appController],
  );

  return (
    <GlobalHotKeys keyMap={hotKeysValue.keyMap} handlers={hotKeysValue.handlers}>
      {buildingMap && (
        <div className={classes.buildingPanel}>
          <ScheduleVisualizer
            buildingMap={buildingMap}
            doorStates={doorStates}
            liftStates={liftStates}
            fleetStates={fleetStates}
            mapFloorSort={mapFloorSort}
            negotiationTrajStore={{}}
            showTrajectories={false}
            onDoorClick={handleDoorMarkerClick}
            onLiftClick={handleLiftMarkerClick}
            onRobotClick={handleRobotMarkerClick}
            onDispenserClick={handleWorkcellMarkerClick}
          ></ScheduleVisualizer>
          <div className={classes.itemPanels}>
            {doors.length > 0 ? (
              <DoorPanel
                doors={doors}
                doorStates={doorStates}
                onDoorControlClick={handleOnDoorControlClick}
              />
            ) : null}
            {lifts.length > 0 ? (
              <LiftPanel
                lifts={lifts}
                liftStates={liftStates}
                onRequestSubmit={handleLiftRequestSubmit}
              />
            ) : null}
            {workcells.length > 0 ? (
              <DispenserPanel dispensers={workcells} dispenserStates={workcellStates} />
            ) : null}
          </div>
        </div>
      )}
    </GlobalHotKeys>
  );
}
