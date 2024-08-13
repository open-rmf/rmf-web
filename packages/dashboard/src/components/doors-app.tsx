import { BuildingMap } from 'api-client';
import React from 'react';
import { DoorDataGridTable, DoorTableData } from 'react-components';
import { DoorMode as RmfDoorMode } from 'rmf-models/ros/rmf_door_msgs/msg/DoorMode';
import { throttleTime } from 'rxjs';

import { getApiErrorMessage } from '../utils/api';
import { AppEvents } from './app-events';
import { createMicroApp } from './micro-app';
import { RmfApiContext } from './rmf-dashboard';

export const DoorsApp = createMicroApp('Doors', () => {
  const rmfApi = React.useContext(RmfApiContext);
  const [buildingMap, setBuildingMap] = React.useState<BuildingMap | null>(null);
  const [doorTableData, setDoorTableData] = React.useState<Record<string, DoorTableData>>({});

  React.useEffect(() => {
    if (!rmfApi) {
      return;
    }
    const sub = rmfApi.buildingMapObs.subscribe(setBuildingMap);
    return () => sub.unsubscribe();
  }, [rmfApi]);

  React.useEffect(() => {
    if (!rmfApi) {
      return;
    }

    let doorIndex = 0;
    buildingMap?.levels.map((level) =>
      level.doors.map(async (door) => {
        try {
          const sub = rmfApi
            .getDoorStateObs(door.name)
            .pipe(throttleTime(3000, undefined, { leading: true, trailing: true }))
            .subscribe((doorState) => {
              setDoorTableData((prev) => {
                return {
                  ...prev,
                  [door.name]: {
                    index: doorIndex++,
                    doorName: door.name,
                    levelName: level.name,
                    doorType: door.door_type,
                    doorState: doorState,
                    onClickOpen: () =>
                      rmfApi?.doorsApi.postDoorRequestDoorsDoorNameRequestPost(door.name, {
                        mode: RmfDoorMode.MODE_OPEN,
                      }),
                    onClickClose: () =>
                      rmfApi?.doorsApi.postDoorRequestDoorsDoorNameRequestPost(door.name, {
                        mode: RmfDoorMode.MODE_CLOSED,
                      }),
                  },
                };
              });
            });
          return () => sub.unsubscribe();
        } catch (error) {
          console.error(`Failed to get lift health: ${getApiErrorMessage(error)}`);
        }
      }),
    );
  }, [rmfApi, buildingMap]);

  return (
    <DoorDataGridTable
      doors={Object.values(doorTableData)}
      onDoorClick={(_ev, doorData) => {
        if (!buildingMap) {
          AppEvents.doorSelect.next(null);
          return;
        }

        for (const level of buildingMap.levels) {
          for (const door of level.doors) {
            if (door.name === doorData.doorName) {
              AppEvents.doorSelect.next([level.name, door]);
              return;
            }
          }
        }
      }}
    />
  );
});
