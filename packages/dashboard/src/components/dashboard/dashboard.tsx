/* istanbul ignore file */

import { Card, Grid, makeStyles } from '@material-ui/core';
import { Fleet, Level } from 'api-client';
import Debug from 'debug';
import React from 'react';
import { DoorData, DoorPanel, LiftPanel, LiftPanelProps, WorkcellPanel } from 'react-components';
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
import ScheduleVisualizer from '../schedule-visualizer';
import {
  useFleets,
  useFleetStateRef,
  useDispenser,
  useIngestor,
} from '../../util/common-subscriptions';

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
    width: 800,
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

  const doorStatesRef = React.useRef<Record<string, RmfModels.DoorState>>({});
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
      sioClient.subscribeDoorState(
        d.door.name,
        (state) => (doorStatesRef.current[d.door.name] = state),
      ),
    );
    return () => {
      subs.forEach((s) => sioClient.unsubscribe(s));
    };
  }, [sioClient, doors]);

  const liftStatesRef = React.useRef<Record<string, RmfModels.LiftState>>({});
  const lifts: RmfModels.Lift[] = React.useMemo(() => (buildingMap ? buildingMap.lifts : []), [
    buildingMap,
  ]);
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
  const dispenserStatesRef = React.useRef<Record<string, RmfModels.DispenserState>>({});
  useDispenser(sioClient, dispensers, dispenserStatesRef);

  const ingestors = React.useContext(IngestorsContext);
  const ingestorStatesRef = React.useRef<Record<string, RmfModels.IngestorState>>({});
  useIngestor(sioClient, ingestors, ingestorStatesRef);

  const workcells = React.useMemo(() => [...dispensers, ...ingestors], [dispensers, ingestors]);
  const workcellStates = { ...dispenserStatesRef.current, ...ingestorStatesRef.current };

  const [fleets, setFleets] = React.useState<Fleet[]>([]);
  useFleets(rmfIngress, setFleets);
  const fleetStatesRef = React.useRef<Record<string, RmfModels.FleetState>>({});
  useFleetStateRef(sioClient, fleets, fleetStatesRef);
  const fleetNames = React.useRef<string[]>([]);
  const newFleetNames = Object.keys(fleetStatesRef.current);
  if (newFleetNames.some((fleetName) => !fleetNames.current.includes(fleetName))) {
    fleetNames.current = newFleetNames;
  }

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
                  workCellStates={workcellStates}
                />
              ) : null}
            </Grid>
          </Grid>
        )}
      </GlobalHotKeys>
    </div>
  );
}
