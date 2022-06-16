import { styled } from '@mui/material';
import { BuildingMap, Dispenser, DoorState, FleetState, Ingestor, LiftState } from 'api-client';
import React from 'react';
import { Subscription } from 'rxjs';
import { AppRegistry, createMicroApp } from './micro-app';
import { RmfIngressContext } from './rmf-app';
import { ScheduleVisualizer } from './schedule-visualizer';

export const MapApp = styled(
  createMicroApp('Map', () => {
    const rmf = React.useContext(RmfIngressContext);
    const [buildingMap, setBuildingMap] = React.useState<BuildingMap | null>(null);
    const [dispensers, setDispensers] = React.useState<Dispenser[]>([]);
    const [ingestors, setIngestors] = React.useState<Ingestor[]>([]);
    const [doorStates, setDoorStates] = React.useState<Record<string, DoorState>>({});
    const [liftStates, setLiftStates] = React.useState<Record<string, LiftState>>({});
    const [fleetStates, setFleetStates] = React.useState<Record<string, FleetState>>({});

    React.useEffect(() => {
      if (!rmf) {
        return;
      }

      const subs: Subscription[] = [];
      subs.push(
        rmf.buildingMapObs.subscribe((newMap) => {
          for (const level of newMap.levels) {
            for (const door of level.doors) {
              subs.push(
                rmf
                  .getDoorStateObs(door.name)
                  .subscribe((state) => setDoorStates((prev) => ({ ...prev, [door.name]: state }))),
              );
            }
          }
          for (const lift of newMap.lifts) {
            subs.push(
              rmf
                .getLiftStateObs(lift.name)
                .subscribe((state) => setLiftStates((prev) => ({ ...prev, [lift.name]: state }))),
            );
          }
          setBuildingMap(newMap);
        }),
      );
      subs.push(rmf.dispensersObs.subscribe(setDispensers));
      subs.push(rmf.ingestorsObs.subscribe(setIngestors));
      subs.push(
        rmf.fleetsObs.subscribe((fleets) => {
          for (const fleet of fleets) {
            const fleetName = fleet.name;
            if (!fleetName) {
              continue;
            }
            subs.push(
              rmf
                .getFleetStateObs(fleetName)
                .subscribe((state) => setFleetStates((prev) => ({ ...prev, [fleetName]: state }))),
            );
          }
        }),
      );

      return () => {
        for (const sub of subs) {
          sub.unsubscribe();
        }
      };
    }, [rmf]);

    return (
      buildingMap && (
        <ScheduleVisualizer
          buildingMap={buildingMap}
          dispensers={dispensers}
          ingestors={ingestors}
          doorStates={doorStates}
          liftStates={liftStates}
          fleetStates={fleetStates}
          mode="normal"
        />
      )
    );
  }),
)({
  // This ensures that the resize handle is above the map.
  '& > .react-resizable-handle': {
    zIndex: 1001,
  },
});

AppRegistry['Map'] = MapApp;
