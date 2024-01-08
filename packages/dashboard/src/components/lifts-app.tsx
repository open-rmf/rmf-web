import { BuildingMap } from 'api-client';
import React from 'react';
import { LiftRequest as RmfLiftRequest } from 'rmf-models';
import { LiftTableData, LiftDataGridTable } from 'react-components';
import { AppEvents } from './app-events';
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
        const sub = rmf.getLiftStateObs(lift.name).subscribe((liftState) => {
          setLiftTableData((prev) => {
            return {
              ...prev,
              [lift.name]: [
                {
                  index: i,
                  name: lift.name,
                  mode: liftState.current_mode,
                  currentFloor: liftState.current_floor,
                  destinationFloor: liftState.destination_floor,
                  doorState: liftState.door_state,
                  motionState: liftState.motion_state,
                  lift: lift,
                  onRequestSubmit: async (_ev, doorState, requestType, destination) => {
                    let fleet_session_ids: string[] = [];
                    if (requestType === RmfLiftRequest.REQUEST_END_SESSION) {
                      const fleets = (await rmf?.fleetsApi.getFleetsFleetsGet()).data;
                      for (const fleet of fleets) {
                        if (!fleet.robots) {
                          continue;
                        }
                        for (const robotName of Object.keys(fleet.robots)) {
                          fleet_session_ids.push(`${fleet.name}/${robotName}`);
                        }
                      }
                    }

                    return rmf?.liftsApi.postLiftRequestLiftsLiftNameRequestPost(lift.name, {
                      destination,
                      door_mode: doorState,
                      request_type: requestType,
                      additional_session_ids: fleet_session_ids,
                    });
                  },
                },
              ],
            };
          });
        });
        return () => sub.unsubscribe();
      } catch (error) {
        console.error(`Failed to get lift state: ${getApiErrorMessage(error)}`);
      }
    });
  }, [rmf, buildingMap]);

  return (
    <LiftDataGridTable
      lifts={Object.values(liftTableData).flatMap((l) => l)}
      onLiftClick={(_ev, liftData) => {
        if (!buildingMap) {
          AppEvents.liftSelect.next(null);
          return;
        }

        for (const lift of buildingMap.lifts) {
          if (lift.name === liftData.name) {
            AppEvents.liftSelect.next(lift);
            return;
          }
        }
      }}
    />
  );
});
