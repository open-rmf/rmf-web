/* istanbul ignore file */

import { Card, Grid, makeStyles } from '@material-ui/core';
import {
  DispenserState,
  Door,
  DoorState,
  Fleet,
  IngestorState,
  Level,
  Lift,
  LiftState,
  FleetState,
} from 'api-client';
import Debug from 'debug';
import React from 'react';
import { DoorData, DoorPanel, LiftPanel, LiftPanelProps, WorkcellPanel } from 'react-components';
import { GlobalHotKeys } from 'react-hotkeys';
import { buildHotKeys } from '../../hotkeys';
import { AppControllerContext } from '../app-contexts';
import {
  BuildingMapContext,
  DispensersContext,
  IngestorsContext,
  RmfIngressContext,
} from '../rmf-app';
import ScheduleVisualizer from '../schedule-visualizer';

const debug = Debug('Dashboard');
const UpdateRate = 1000;

const useStyles = makeStyles((theme) => ({
  root: {
    height: '100%',
    backgroundColor: theme.palette.background.default,
  },
  toolBarTitle: {
    flexGrow: 1,
  },
  buildingPanel: {
    height: '100%',
  },
  mapPanel: {
    margin: theme.spacing(1),
    flex: '1 0 auto',
  },
  itemPanels: {
    width: '25%',
  },
}));

export default function Dashboard(_props: {}): React.ReactElement {
  debug('render');

  const classes = useStyles();
  const appController = React.useContext(AppControllerContext);
  const rmfIngress = React.useContext(RmfIngressContext);
  const sioClient = rmfIngress?.sioClient;
  const buildingMap = React.useContext(BuildingMapContext);

  const [_triggerRender, setTriggerRender] = React.useState(0); // eslint-disable-line @typescript-eslint/no-unused-vars
  React.useEffect(() => {
    const interval = setInterval(() => setTriggerRender((prev) => prev + 1), UpdateRate);
    return () => clearInterval(interval);
  }, []);

  const doorStatesRef = React.useRef<Record<string, DoorState>>({});
  const doors: DoorData[] = React.useMemo(() => {
    return buildingMap
      ? (buildingMap.levels as Level[]).flatMap((x) =>
          (x.doors as Door[]).map((door) => ({ door, level: x.name })),
        )
      : [];
  }, [buildingMap]);

  React.useEffect(() => {
    if (!sioClient) return;
    const subs = doors.map((d) =>
      sioClient.subscribeDoorState(
        d.door.name,
        (state) => (doorStatesRef.current[d.door.name] = state),
      ),
    );
    return () => {
      subs.forEach((s) => sioClient.unsubscribe(s));
    };
  }, [sioClient, doors]);

  const liftStatesRef = React.useRef<Record<string, LiftState>>({});
  const lifts: Lift[] = React.useMemo(() => (buildingMap ? buildingMap.lifts : []), [buildingMap]);
  React.useEffect(() => {
    if (!sioClient) return;
    const subs = lifts.map((l) =>
      sioClient.subscribeLiftState(l.name, (state) => (liftStatesRef.current[l.name] = state)),
    );
    return () => {
      subs.forEach((s) => sioClient.unsubscribe(s));
    };
  }, [sioClient, lifts]);

  const dispensers = React.useContext(DispensersContext);
  const dispenserStatesRef = React.useRef<Record<string, DispenserState>>({});
  React.useEffect(() => {
    if (!sioClient) return;
    const subs = dispensers.map((d) =>
      sioClient.subscribeDispenserState(
        d.guid,
        (state) => (dispenserStatesRef.current[d.guid] = state),
      ),
    );
    return () => {
      subs.forEach((s) => sioClient.unsubscribe(s));
    };
  }, [sioClient, dispensers]);

  const ingestors = React.useContext(IngestorsContext);
  const ingestorStatesRef = React.useRef<Record<string, IngestorState>>({});
  React.useEffect(() => {
    if (!sioClient) return;
    const subs = ingestors.map((d) =>
      sioClient.subscribeIngestorState(
        d.guid,
        (state) => (ingestorStatesRef.current[d.guid] = state),
      ),
    );
    return () => {
      subs.forEach((s) => sioClient.unsubscribe(s));
    };
  }, [sioClient, ingestors]);

  const workcells = React.useMemo(() => [...dispensers, ...ingestors], [dispensers, ingestors]);
  const workcellStates = { ...dispenserStatesRef.current, ...ingestorStatesRef.current };

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
  const fleetStatesRef = React.useRef<Record<string, FleetState>>({});
  React.useEffect(() => {
    if (!sioClient) return;
    const subs = fleets.map((f) =>
      sioClient.subscribeFleetState(f.name, (state) => (fleetStatesRef.current[f.name] = state)),
    );
    return () => {
      subs.forEach((s) => sioClient.unsubscribe(s));
    };
  }, [sioClient, fleets]);
  const fleetNames = React.useRef<string[]>([]);
  const newFleetNames = Object.keys(fleetStatesRef.current);
  if (newFleetNames.some((fleetName) => !fleetNames.current.includes(fleetName))) {
    fleetNames.current = newFleetNames;
  }

  const { doorsApi, liftsApi } = React.useContext(RmfIngressContext) || {};

  const handleOnDoorControlClick = React.useCallback(
    (_ev, door: Door, mode: number) =>
      doorsApi?.postDoorRequestDoorsDoorNameRequestPost(door.name, {
        mode: mode,
      }),
    [doorsApi],
  );

  const handleLiftRequestSubmit = React.useCallback<Required<LiftPanelProps>['onRequestSubmit']>(
    (_ev, lift, doorState, requestType, destination) =>
      liftsApi?.postLiftRequestLiftsLiftNameRequestPost(lift.name, {
        destination,
        request_type: requestType,
        door_mode: doorState,
      }),
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
    <div className={classes.root}>
      <GlobalHotKeys keyMap={hotKeysValue.keyMap} handlers={hotKeysValue.handlers}>
        {buildingMap && (
          <Grid container className={classes.buildingPanel} wrap="nowrap">
            <Card variant="outlined" className={classes.mapPanel}>
              <ScheduleVisualizer
                buildingMap={buildingMap}
                dispensers={dispensers}
                ingestors={ingestors}
                doorStates={Object.assign({}, doorStatesRef.current)}
                liftStates={Object.assign({}, liftStatesRef.current)}
                fleetStates={Object.assign({}, fleetStatesRef.current)}
                mode="normal"
              ></ScheduleVisualizer>
            </Card>
            <Grid item className={classes.itemPanels}>
              {doors.length > 0 ? (
                <DoorPanel
                  doors={doors}
                  doorStates={doorStatesRef.current}
                  onDoorControlClick={handleOnDoorControlClick}
                />
              ) : null}
              {lifts.length > 0 ? (
                <LiftPanel
                  lifts={lifts}
                  liftStates={liftStatesRef.current}
                  onRequestSubmit={handleLiftRequestSubmit}
                />
              ) : null}
              {workcells.length > 0 ? (
                <WorkcellPanel
                  dispensers={dispensers}
                  ingestors={ingestors}
                  workcellStates={workcellStates}
                />
              ) : null}
            </Grid>
          </Grid>
        )}
      </GlobalHotKeys>
    </div>
  );
}
