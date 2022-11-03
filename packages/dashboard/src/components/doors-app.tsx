import { CardActions, Grid } from '@mui/material';
import { BuildingMap, DoorState } from 'api-client';
import React from 'react';
import {
  DoorCard as BaseDoorCard,
  DoorCardProps as BaseDoorCardProps,
  DoorControls,
} from 'react-components';
import { DoorMode as RmfDoorMode } from 'rmf-models';
import { createMicroApp } from './micro-app';
import { RmfAppContext } from './rmf-app';

type DoorCardProps = Omit<BaseDoorCardProps, 'mode'>;

const DoorCard = ({ children, ...otherProps }: DoorCardProps) => {
  const rmf = React.useContext(RmfAppContext);
  const [doorState, setDoorState] = React.useState<DoorState | null>(null);
  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = rmf.getDoorStateObs(otherProps.name).subscribe(setDoorState);
    return () => sub.unsubscribe();
  }, [rmf, otherProps.name]);

  return (
    <BaseDoorCard mode={doorState ? doorState.current_mode.value : -1} {...otherProps}>
      {children}
    </BaseDoorCard>
  );
};

export const DoorsApp = createMicroApp('Doors', () => {
  const rmf = React.useContext(RmfAppContext);
  const [buildingMap, setBuildingMap] = React.useState<BuildingMap | null>(null);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = rmf.buildingMapObs.subscribe(setBuildingMap);
    return () => sub.unsubscribe();
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
              type={door.door_type}
              sx={{ width: 200 }}
              aria-labelledby={`door-cell-${door.name}`}
            >
              <CardActions sx={{ justifyContent: 'center' }}>
                <DoorControls
                  doorName={door.name}
                  onOpenClick={() =>
                    rmf?.doorsApi.postDoorRequestDoorsDoorNameRequestPost(door.name, {
                      mode: RmfDoorMode.MODE_OPEN,
                    })
                  }
                  onCloseClick={() =>
                    rmf?.doorsApi.postDoorRequestDoorsDoorNameRequestPost(door.name, {
                      mode: RmfDoorMode.MODE_CLOSED,
                    })
                  }
                />
              </CardActions>
            </DoorCard>
          )),
        )}
    </Grid>
  );
});
