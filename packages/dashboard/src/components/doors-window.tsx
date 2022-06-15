import { CardActions, Grid } from '@mui/material';
import React from 'react';
import { DoorCard as DoorCard_, DoorControls, createWindow } from 'react-components';
import { Door, DoorState } from 'rmf-models';
import { RmfIngressContext } from './rmf-app';

const DoorCard = React.memo(DoorCard_);

export const DoorsWindow = createWindow(() => {
  const rmf = React.useContext(RmfIngressContext);
  const [doors, setDoors] = React.useState<Door[]>([]);
  const [doorStates, setDoorStates] = React.useState<Record<string, DoorState>>({});

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    (async () => {
      let newDoors = await rmf.getDoors();
      for (const door of newDoors) {
        rmf.getDoorStateObs(door.name).subscribe((state) => {
          setDoorStates((prev) => ({ ...prev, [door.name]: state }));
        });
      }
      setDoors(newDoors);
    })();
  }, [rmf]);

  return (
    <Grid container>
      {doors.map((door) => (
        <DoorCard
          key={door.name}
          name={door.name}
          level="L1"
          mode={doorStates[door.name] === undefined ? -1 : doorStates[door.name].current_mode.value}
          type={door.door_type}
          sx={{ width: 200 }}
        >
          <CardActions sx={{ justifyContent: 'center' }}>
            <DoorControls />
          </CardActions>
        </DoorCard>
      ))}
    </Grid>
  );
});
