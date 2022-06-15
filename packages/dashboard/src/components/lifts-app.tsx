import { CardActions, Grid } from '@mui/material';
import { BuildingMap, LiftState } from 'api-client';
import React from 'react';
import { LiftCard as LiftCard_, LiftControls as LiftControls_ } from 'react-components';
import { AppRegistry, createMicroApp } from './micro-app';
import { RmfIngressContext } from './rmf-app';

const LiftCard = React.memo(LiftCard_);
const LiftControls = React.memo(LiftControls_);

export const LiftsApp = createMicroApp('Lifts', () => {
  const rmf = React.useContext(RmfIngressContext);
  const [buildingMap, setBuildingMap] = React.useState<BuildingMap | null>(null);
  const [liftStates, setLiftStates] = React.useState<Record<string, LiftState>>({});

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    rmf.buildingMapObs.subscribe((newMap) => {
      for (const lift of newMap.lifts) {
        rmf.getLiftStateObs(lift.name).subscribe((state) => {
          setLiftStates((prev) => ({ ...prev, [lift.name]: state }));
        });
      }
      setBuildingMap(newMap);
    });
  }, [rmf]);

  return (
    <Grid container>
      {buildingMap &&
        buildingMap.lifts.map((lift) => {
          const liftState: LiftState | undefined = liftStates[lift.name];
          return (
            <LiftCard
              key={lift.name}
              name={lift.name}
              motionState={liftState?.motion_state}
              doorState={liftState?.door_state}
              currentFloor={liftState?.current_floor}
              destinationFloor={liftState?.destination_floor}
              sx={{ width: 200 }}
            >
              <CardActions sx={{ justifyContent: 'center' }}>
                <LiftControls
                  availableLevels={lift.levels}
                  currentLevel={liftState?.current_floor}
                />
              </CardActions>
            </LiftCard>
          );
        })}
    </Grid>
  );
});

AppRegistry['Lifts'] = LiftsApp;
