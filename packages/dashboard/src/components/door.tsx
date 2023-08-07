import * as THREE from 'three';
import React from 'react';
import { DoorState, Lift, LiftState } from 'api-client';
import { Cube } from './cube';
import { Door as DoorModel } from 'rmf-models';
import { RmfAppContext } from './rmf-app';
import { DoorMode } from 'rmf-models';

function distance(v1_x: number, v1_y: number, v2_x: number, v2_y: number) {
  return Math.hypot(v2_x - v1_x, v2_y - v1_y);
}

function midPoint(v1_x: number, v1_y: number, v2_x: number, v2_y: number) {
  return [(v2_x + v1_x) / 2, (v2_y + v1_y) / 2];
}

interface DoorProps {
  door: DoorModel;
  opacity: number;
  height: number;
  elevation: number;
  lift?: Lift;
}

interface SingleDoorProps extends DoorProps {
  door: DoorModel;
  opacity: number;
  meshRef: React.Ref<THREE.Mesh>;
  doorState: number | undefined;
  color: string;
}

function SingleSwingDoor({
  meshRef,
  opacity,
  door,
  height,
  doorState,
  elevation,
  color,
}: SingleDoorProps) {
  const { v1_x, v1_y, v2_x, v2_y } = door;
  const thickness = 0.5;
  const v = new THREE.Vector3(v1_x - v2_x, 0, v1_y - v2_y);
  v.normalize();
  const angle = Math.atan2(v1_y - v2_y, v1_x - v2_x) - Math.PI / 2;
  const rot = new THREE.Euler(0, 0, angle);

  const pos = midPoint(v1_x, v1_y, v2_x, v2_y).concat(height / 2 + elevation);
  const dist = distance(v1_x, v1_y, v2_x, v2_y);
  return (
    <Cube
      meshRef={meshRef}
      key={door.name}
      position={pos}
      size={[thickness, dist, height]}
      rot={rot}
      color={color}
    />
  );
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
    if (!rmf) {
      return;
    }
    const sub = rmf.getDoorStateObs(door.name).subscribe(setDoorState);
    return () => sub.unsubscribe();
  }, [rmf, door.name]);

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

  return (
    <SingleSwingDoor
      {...doorProps}
      key={door.name}
      meshRef={ref}
      doorState={doorState?.current_mode.value}
      color={color}
    />
  );
});
