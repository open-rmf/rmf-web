import { TableContainer } from '@mui/material';
import { BuildingMap, Lift } from 'api-client';
import React from 'react';
import { LiftDataGridTable, LiftTableData } from 'react-components';
import { LiftRequest as RmfLiftRequest } from 'rmf-models/ros/rmf_lift_msgs/msg';
import { throttleTime } from 'rxjs';

import { AppEvents } from './app-events';
import { LiftSummary } from './lift-summary';
import { createMicroApp } from './micro-app';
import { RmfAppContext } from './rmf-app';
import { getApiErrorMessage } from './utils';

export const LiftsApp = createMicroApp('Lifts', () => {
  const rmf = React.useContext(RmfAppContext);
  const [buildingMap, setBuildingMap] = React.useState<BuildingMap | null>(null);
  const [liftTableData, setLiftTableData] = React.useState<Record<string, LiftTableData[]>>({});
  const [openLiftSummary, setOpenLiftSummary] = React.useState(false);
  const [selectedLift, setSelectedLift] = React.useState<Lift | null>(null);

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
        const sub = rmf
          .getLiftStateObs(lift.name)
          .pipe(throttleTime(3000, undefined, { leading: true, trailing: true }))
          .subscribe((liftState) => {
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
                    sessionId: liftState.session_id,
                    lift: lift,
                    liftState: liftState,
                    onRequestSubmit: async (_ev, doorState, requestType, destination) => {
                      if (!rmf) {
                        console.error('rmf ingress is undefined');
                        return;
                      }
                      const fleet_session_ids: string[] = [];
                      if (requestType === RmfLiftRequest.REQUEST_END_SESSION) {
                        const fleets = (await rmf.fleetsApi.getFleetsFleetsGet()).data;
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
    <TableContainer>
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
              setSelectedLift(lift);
              setOpenLiftSummary(true);
              return;
            }
          }
        }}
      />
      {openLiftSummary && selectedLift && (
        <LiftSummary lift={selectedLift} onClose={() => setOpenLiftSummary(false)} />
      )}
    </TableContainer>
  );
});
