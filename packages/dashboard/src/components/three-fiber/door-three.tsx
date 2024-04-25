import { ThreeEvent } from '@react-three/fiber';
import { DoorState, Lift, LiftState } from 'api-client';
import React from 'react';
import { DoorThreeMaker } from 'react-components';
import { DoorMode, Door as DoorModel } from 'rmf-models';
import { throttleTime } from 'rxjs';
import { RmfAppContext } from '../rmf-app';

interface DoorProps {
  door: DoorModel;
  opacity: number;
  height: number;
  elevation: number;
  lift?: Lift;
  onDoorClick?: (ev: ThreeEvent<MouseEvent>, door: DoorModel) => void;
}

function toDoorMode(liftState: LiftState): DoorMode {
  return { value: liftState.door_state };
}

export const Door = React.memo(({ ...doorProps }: DoorProps): JSX.Element => {
  const ref = React.useRef<THREE.Mesh>(null!);
  const { door, lift, onDoorClick } = doorProps;
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

    const sub = rmf
      .getLiftStateObs(lift.name)
      .pipe(throttleTime(3000, undefined, { leading: true, trailing: true }))
      .subscribe(setLiftState);
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

  return (
    <DoorThreeMaker
      {...doorProps}
      height={8}
      key={door.name}
      meshRef={ref}
      color={color}
      onDoorClick={(ev: ThreeEvent<MouseEvent>) => onDoorClick && onDoorClick(ev, door)}
    />
  );
});
