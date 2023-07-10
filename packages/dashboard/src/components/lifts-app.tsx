import { BuildingMap } from 'api-client';
import React from 'react';
import { LiftTableData, LiftDataGridTable } from 'react-components';
import { createMicroApp } from './micro-app';
import { RmfAppContext } from './rmf-app';
import { getApiErrorMessage } from './utils';

export const LiftsApp = createMicroApp('Lifts', () => {
  const rmf = React.useContext(RmfAppContext);
  const [buildingMap, setBuildingMap] = React.useState<BuildingMap | null>(null);
  const [liftTableData, setLiftTableData] = React.useState<Record<string, LiftTableData[]>>({});

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

    buildingMap?.lifts.map(async (lift, i) => {
      try {
        const { data } = await rmf.liftsApi.getLiftHealthLiftsLiftNameHealthGet(lift.name);
        const { health_status } = data;
        const sub = rmf.getLiftStateObs(lift.name).subscribe((liftState) => {
          setLiftTableData((prev) => {
            return {
              ...prev,
              [lift.name]: [
                {
                  index: i,
                  name: lift.name,
                  opMode: health_status ? health_status : 'N/A',
                  currentFloor: liftState.current_floor,
                  destinationFloor: liftState.destination_floor,
                  doorState: liftState.door_state,
                  motionState: liftState.motion_state,
                  lift: lift,
                  onRequestSubmit: (_ev, doorState, requestType, destination) => {
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
      } catch (error) {
        throw new Error(getApiErrorMessage(error));
      }
    });
  }, [rmf, buildingMap]);

  return <LiftDataGridTable lifts={Object.values(liftTableData).flatMap((l) => l)} />;
});
