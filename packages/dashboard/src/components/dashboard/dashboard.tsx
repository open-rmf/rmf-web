/* istanbul ignore file */

import { Card, Grid, styled } from '@mui/material';
import { Door, DoorState, FleetState, Level, Lift, LiftState } from 'api-client';
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
import {
  useFleets,
  useFleetStateRef,
  useDispenserStatesRef,
  useIngestorStatesRef,
} from '../../util/common-subscriptions';

const debug = Debug('Dashboard');
const UpdateRate = 1000;

const prefix = 'dashboard';
const classes = {
  root: `${prefix}-root`,
  buildingPanel: `${prefix}-building-panel`,
  mapPanel: `${prefix}-map-panel`,
  itemPanels: `${prefix}-item-panels`,
};
const StyledDiv = styled('div')(({ theme }) => ({
  [`&.${classes.root}`]: {
    height: '100%',
    backgroundColor: theme.palette.background.default,
  },
  [`& .${classes.buildingPanel}`]: {
    height: '100vh',
  },
  [`& .${classes.mapPanel}`]: {
    margin: theme.spacing(1),
    flex: '1 0 auto',
  },
  [`& .${classes.itemPanels}`]: {
    width: '55%',
  },
}));

export default function Dashboard(_props: {}): React.ReactElement {
  debug('render');

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
          (x.doors as Door[]).map((door) => ({ level: x.name, door } as DoorData)),
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
  const dispenserStatesRef = useDispenserStatesRef(sioClient, dispensers);

  const ingestors = React.useContext(IngestorsContext);
  const ingestorStatesRef = useIngestorStatesRef(sioClient, ingestors);

  const workcells = React.useMemo(() => [...dispensers, ...ingestors], [dispensers, ingestors]);
  const workcellStates = { ...dispenserStatesRef.current, ...ingestorStatesRef.current };

  const [fleets, setFleets] = React.useState<FleetState[]>([]);
  useFleets(rmfIngress, setFleets);
  const fleetStatesRef = useFleetStateRef(sioClient, fleets);

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
    <StyledDiv className={classes.root}>
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
    </StyledDiv>
  );
}
