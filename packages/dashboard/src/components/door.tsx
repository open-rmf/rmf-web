import * as THREE from 'three';
import React, { Ref, useRef, useState } from 'react';
import { Canvas, useFrame, MeshProps } from '@react-three/fiber';
import { Euler, StringKeyframeTrack } from 'three';
import { DoorState, GraphNode } from 'api-client';
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
}

interface SingleDoorProps extends DoorProps {
  door: DoorModel;
  opacity: number;
  meshRef: React.Ref<THREE.Mesh>;
  doorState: number | undefined;
}

function SingleSwingDoor({
  meshRef,
  opacity,
  door,
  height,
  doorState,
  elevation,
}: SingleDoorProps) {
  const { v1_x, v1_y, v2_x, v2_y, door_type, motion_direction, name } = door;
  const thickness = 0.5;
  const v = new THREE.Vector3(v1_x - v2_x, 0, v1_y - v2_y);
  v.normalize();
  const angle = Math.atan2(v1_y - v2_y, v1_x - v2_x) - Math.PI / 2;
  const rot = new THREE.Euler(0, 0, angle);
  let z = v.angleTo(new THREE.Vector3(0, 0, 1));

  if (doorState === 1) {
    z += 1;
  } else if (doorState === 2) {
    z += 3;
  }

  const pos = midPoint(v1_x, v1_y, v2_x, v2_y).concat(height / 2 + elevation);
  const dist = distance(v1_x, v1_y, v2_x, v2_y);
  return (
    <Cube
      meshRef={meshRef}
      key={door.name}
      position={pos}
      size={[thickness, dist, height]}
      rot={rot}
      color={'red'}
    />
  );
}

export const Door = React.memo(({ ...doorProps }: DoorProps): JSX.Element => {
  const ref = useRef<THREE.Mesh>(null!);
  const { door } = doorProps;
  const rmf = React.useContext(RmfAppContext);
  const [doorState, setDoorState] = React.useState<DoorState | null>(null);

  React.useEffect(() => {
    if (!rmf) {
      // console.log("rmf null")
      return;
    }
    // console.log("In useEffect door")
    // console.log(door.name);
    const sub = rmf.getDoorStateObs(door.name).subscribe(setDoorState);
    return () => sub.unsubscribe();
  }, [rmf, door.name]);

  //   console.log(doorState);
  //   useFrame(() => {
  //     if (ref.current) {
  //       if (doorState?.current_mode.value === 1) {
  //         ref.current.rotation.z += 0.1;
  //       }
  //     }
  //   });

  // switch (doorState?.current_mode.value){
  //   case DoorMode.MODE_CLOSED:
  //     return <SingleSwingDoorClosed {...doorVisProps} {...doorState} />
  //   case DoorMode.MODE_OPEN:
  //     ref.current.rotateZ(0.5);
  //     return <SingleSwingDoorOpened {...doorVisProps} {...doorState} />
  //   default:
  //     return null;
  // }

  // switch (doorState?.current_mode.value){
  //   case DoorMode.MODE_CLOSED:
  //     break;
  //   case DoorMode.MODE_OPEN:
  //     ref.current.rotateZ(0.5);
  //     break;
  //   case DoorMode.MODE_MOVING:
  //     ref.current.rotateY(0.5);
  //     break;
  //   }

  return (
    <SingleSwingDoor
      {...doorProps}
      key={door.name}
      meshRef={ref}
      doorState={doorState?.current_mode.value}
    />
  );
  // switch (door.door_type) {
  //   case Door.DOOR_TYPE_DOUBLE_SWING:
  //     return <SingleSwingDoor {...doorVisProps} />
  //   default:
  //     return null;
  // }
});
