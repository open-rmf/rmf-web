import { Dispenser, Fleet, Ingestor } from 'api-client';
import React from 'react';
import { useAsync, Window } from 'react-components';
import * as RmfModels from 'rmf-models';
import { throttleTime } from 'rxjs';
import { RmfIngressContext, RxRmfContext } from '../rmf-app';
import { allWindows, ManagedWindowProps, WindowClass } from '../window';
import ScheduleVisualizer from './schedule-visualizer';

export const ScheduleVisualizerWindow: React.FC<ManagedWindowProps> = React.forwardRef(
  ({ ...otherProps }, ref) => {
    const safeAsync = useAsync();
    const rmfIngress = React.useContext(RmfIngressContext);
    const rxRmf = React.useContext(RxRmfContext);

    const [buildingMap, setBuildingMap] = React.useState<RmfModels.BuildingMap | null>(null);
    React.useEffect(() => {
      if (!rmfIngress) return;
      (async () => {
        const buildingMap = (await safeAsync(rmfIngress.buildingApi.getBuildingMapBuildingMapGet()))
          .data as RmfModels.BuildingMap;
        setBuildingMap(buildingMap);
      })();
    }, [safeAsync, rmfIngress]);

    const [dispensers, setDispensers] = React.useState<Dispenser[]>([]);
    React.useEffect(() => {
      if (!rmfIngress) return;

      (async () => {
        const dispensers = (await safeAsync(rmfIngress.dispensersApi.getDispensersDispensersGet()))
          .data as Dispenser[];
        setDispensers(dispensers);
      })();
    }, [safeAsync, rmfIngress]);

    const [ingestors, setIngestors] = React.useState<Ingestor[]>([]);
    React.useEffect(() => {
      if (!rmfIngress) return;

      (async () => {
        const ingestors = (await safeAsync(rmfIngress.ingestorsApi.getIngestorsIngestorsGet()))
          .data as Ingestor[];
        setIngestors(ingestors);
      })();
    }, [safeAsync, rmfIngress]);

    const [doorStates, setDoorStates] = React.useState<Record<string, RmfModels.DoorState>>({});
    React.useEffect(() => {
      if (!rxRmf || !buildingMap) return;
      const doors = buildingMap.levels.flatMap((level) => level.doors);
      const subs = doors.map((door) =>
        rxRmf
          .doorStates(door.name)
          .pipe(throttleTime(1000))
          .subscribe(
            (doorState) =>
              doorState && setDoorStates((prev) => ({ ...prev, [door.name]: doorState })),
          ),
      );
      return () => {
        subs.forEach((sub) => sub.unsubscribe());
      };
    }, [rxRmf, buildingMap]);

    const [liftStates, setLiftStates] = React.useState<Record<string, RmfModels.LiftState>>({});
    React.useEffect(() => {
      if (!rxRmf || !buildingMap) return;
      const subs = buildingMap.lifts.map((lift) =>
        rxRmf
          .liftStates(lift.name)
          .pipe(throttleTime(1000))
          .subscribe(
            (liftState) =>
              liftState && setLiftStates((prev) => ({ ...prev, [lift.name]: liftState })),
          ),
      );
      return () => {
        subs.forEach((sub) => sub.unsubscribe());
      };
    }, [rxRmf, buildingMap]);

    const [fleets, setFleets] = React.useState<Fleet[]>([]);
    React.useEffect(() => {
      if (!rmfIngress) return;
      (async () => {
        const fleets = (await safeAsync(rmfIngress.fleetsApi.getFleetsFleetsGet())).data;
        setFleets(fleets);
      })();
    }, [safeAsync, rmfIngress]);

    const [fleetStates, setFleetStates] = React.useState<Record<string, RmfModels.FleetState>>({});
    React.useEffect(() => {
      if (!rxRmf) return;
      fleets.map((fleet) =>
        rxRmf
          .fleetStates(fleet.name)
          .pipe(throttleTime(1000))
          .subscribe(
            (fleetState) =>
              fleetState && setFleetStates((prev) => ({ ...prev, [fleet.name]: fleetState })),
          ),
      );
    }, [rxRmf, fleets]);

    return (
      <Window ref={ref} title="Map" {...otherProps}>
        {buildingMap && (
          <ScheduleVisualizer
            buildingMap={buildingMap}
            dispensers={dispensers}
            ingestors={ingestors}
            doorStates={doorStates}
            liftStates={liftStates}
            fleetStates={fleetStates}
          />
        )}
      </Window>
    );
  },
);

export default ScheduleVisualizerWindow;

export const scheduleVisualizerWindowClass = new WindowClass(
  'Interactive Map',
  ScheduleVisualizerWindow,
  {
    x: 0,
    y: 0,
    w: 4,
    h: 4,
    minW: 4,
    minH: 4,
  },
);

allWindows['scheduleVisualizer'] = scheduleVisualizerWindowClass;
