import { ThreeEvent } from '@react-three/fiber';
import { DoorState, Lift, LiftState } from 'api-client';
import React from 'react';
import { DoorThreeMaker } from 'react-components';
import { Door as RmfDoor } from 'rmf-models/ros/rmf_building_map_msgs/msg';
import { DoorMode as RmfDoorMode } from 'rmf-models/ros/rmf_door_msgs/msg';
import { throttleTime } from 'rxjs';
import { Mesh } from 'three';

import { RmfApiContext } from '../rmf-dashboard';

interface DoorProps {
  door: RmfDoor;
  opacity: number;
  height: number;
  elevation: number;
  lift?: Lift;
  onDoorClick?: (ev: ThreeEvent<MouseEvent>, door: RmfDoor) => void;
}

function toDoorMode(liftState: LiftState): RmfDoorMode {
  return { value: liftState.door_state };
}

export const Door = React.memo(({ ...doorProps }: DoorProps): JSX.Element => {
  const ref = React.useRef<Mesh>(null!);
  const { door, lift, onDoorClick } = doorProps;
  const rmfApi = React.useContext(RmfApiContext);
  const [doorState, setDoorState] = React.useState<DoorState | null>(null);
  const [liftState, setLiftState] = React.useState<LiftState | undefined>(undefined);
  const [color, setColor] = React.useState<string>('red');

  React.useEffect(() => {
    if (!rmfApi) {
      return;
    }
    if (!lift) {
      return;
    }

    const sub = rmfApi
      .getLiftStateObs(lift.name)
      .pipe(throttleTime(3000, undefined, { leading: true, trailing: true }))
      .subscribe(setLiftState);
    return () => sub.unsubscribe();
  }, [rmfApi, lift]);

  React.useEffect(() => {
    let doorStateValue = doorState?.current_mode.value;
    if (liftState) {
      doorStateValue = toDoorMode(liftState).value;
    }
    switch (doorStateValue) {
      case RmfDoorMode.MODE_CLOSED:
        setColor('red');
        return;
      case RmfDoorMode.MODE_MOVING:
        setColor('orange');
        return;
      case RmfDoorMode.MODE_OPEN:
      default:
        setColor('green');
        return;
    }
  }, [doorState?.current_mode.value, liftState]);

  React.useEffect(() => {
    if (!rmfApi) {
      return;
    }
    const sub = rmfApi.getDoorStateObs(door.name).subscribe(setDoorState);
    return () => sub.unsubscribe();
  }, [rmfApi, door.name]);

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
