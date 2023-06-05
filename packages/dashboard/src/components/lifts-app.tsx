import { BuildingMap } from 'api-client';
import React from 'react';
import { LiftData, LiftDataGridTable } from 'react-components';
import { createMicroApp } from './micro-app';
import { RmfAppContext } from './rmf-app';

export const LiftsApp = createMicroApp('Lifts', () => {
  const rmf = React.useContext(RmfAppContext);
  const [buildingMap, setBuildingMap] = React.useState<BuildingMap | null>(null);
  const [data, setData] = React.useState<Record<string, LiftData[]>>({});

  React.useEffect(() => {
    if (!rmf) {
      return;
    }

    const sub = rmf.buildingMapObs.subscribe((newMap) => {
      setBuildingMap(newMap);
    });
    return () => sub.unsubscribe();
  }, [rmf]);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }

    buildingMap?.lifts.map((lift, i) => {
      const sub = rmf.getLiftStateObs(lift.name).subscribe((liftState) => {
        setData((prev) => {
          return {
            ...prev,
            [lift.name]: [
              {
                index: i,
                name: lift.name,
                current_floor: liftState.current_floor,
                destination_floor: liftState.destination_floor,
                door_state: liftState.door_state,
                motion_state: liftState.motion_state,
                lift: lift,
                onRequestSubmit: (_ev, doorState, requestType, destination) => {
                  console.log(lift.name);
                  return rmf?.liftsApi.postLiftRequestLiftsLiftNameRequestPost(lift.name, {
                    destination,
                    door_mode: doorState,
                    request_type: requestType,
                  });
                },
              },
            ],
          };
        });
      });
      return () => sub.unsubscribe();
    });
  }, [rmf, buildingMap]);

  return <LiftDataGridTable lifts={Object.values(data).flatMap((r) => r)} />;
});
