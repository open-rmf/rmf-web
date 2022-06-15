import { CardActions, Grid } from '@mui/material';
import { BuildingMap, DoorState } from 'api-client';
import React from 'react';
import { DoorCard as DoorCard_, DoorControls } from 'react-components';
import { AppRegistry, createMicroApp } from './micro-app';
import { RmfIngressContext } from './rmf-app';

const DoorCard = React.memo(DoorCard_);

export const DoorsApp = createMicroApp('Doors', () => {
  const rmf = React.useContext(RmfIngressContext);
  const [buildingMap, setBuildingMap] = React.useState<BuildingMap | null>(null);
  const [doorStates, setDoorStates] = React.useState<Record<string, DoorState>>({});

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    rmf.buildingMapObs.subscribe((newMap) => {
      for (const door of newMap.levels.flatMap((level) => level.doors)) {
        rmf.getDoorStateObs(door.name).subscribe((state) => {
          setDoorStates((prev) => ({ ...prev, [door.name]: state }));
        });
      }
      setBuildingMap(newMap);
    });
  }, [rmf]);

  return (
    <Grid container>
      {buildingMap &&
        buildingMap.levels.flatMap((level) =>
          level.doors.map((door) => (
            <DoorCard
              key={door.name}
              name={door.name}
              level={level.name}
              mode={
                doorStates[door.name] === undefined ? -1 : doorStates[door.name].current_mode.value
              }
              type={door.door_type}
              sx={{ width: 200 }}
            >
              <CardActions sx={{ justifyContent: 'center' }}>
                <DoorControls />
              </CardActions>
            </DoorCard>
          )),
        )}
    </Grid>
  );
});

AppRegistry['Doors'] = DoorsApp;
