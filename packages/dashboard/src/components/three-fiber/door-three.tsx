import React from 'react';
import { DoorState, Lift, LiftState } from 'api-client';
import { Door as DoorModel } from 'rmf-models';
import { RmfAppContext } from '../rmf-app';
import { DoorMode } from 'rmf-models';
import { DoorThreeMaker } from 'react-components';

interface DoorProps {
  door: DoorModel;
  opacity: number;
  height: number;
  elevation: number;
  lift?: Lift;
}

function toDoorMode(liftState: LiftState): DoorMode {
  return { value: liftState.door_state };
}

export const Door = React.memo(({ ...doorProps }: DoorProps): JSX.Element => {
  const ref = React.useRef<THREE.Mesh>(null!);
  const { door, lift } = doorProps;
  const rmf = React.useContext(RmfAppContext);
  const [doorState, setDoorState] = React.useState<DoorState | null>(null);
  const [liftState, setLiftState] = React.useState<LiftState | undefined>(undefined);
  const [color, setColor] = React.useState<string>('red');

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    if (!lift) {
      return;
    }

    const sub = rmf.getLiftStateObs(lift.name).subscribe(setLiftState);
    return () => sub.unsubscribe();
  }, [rmf, lift]);

  React.useEffect(() => {
    let doorStateValue = doorState?.current_mode.value;
    if (liftState) {
      doorStateValue = toDoorMode(liftState).value;
    }
    switch (doorStateValue) {
      case DoorMode.MODE_CLOSED:
        setColor('red');
        return;
      case DoorMode.MODE_MOVING:
        setColor('orange');
        return;
      case DoorMode.MODE_OPEN:
      default:
        setColor('green');
        return;
    }
  }, [doorState?.current_mode.value, liftState]);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = rmf.getDoorStateObs(door.name).subscribe(setDoorState);
    return () => sub.unsubscribe();
  }, [rmf, door.name]);

  return <DoorThreeMaker {...doorProps} height={8} key={door.name} meshRef={ref} color={color} />;
});
