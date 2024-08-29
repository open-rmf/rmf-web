import { ThreeEvent } from '@react-three/fiber';
import React from 'react';
import { Door as DoorModel } from 'rmf-models/ros/rmf_building_map_msgs/msg';
import { Euler, Mesh } from 'three';

import { CubeMaker } from './cube-maker';

const distance = (v1_x: number, v1_y: number, v2_x: number, v2_y: number) =>
  Math.hypot(v2_x - v1_x, v2_y - v1_y);
const midPoint = (v1_x: number, v1_y: number, v2_x: number, v2_y: number) => [
  (v2_x + v1_x) / 2,
  (v2_y + v1_y) / 2,
];

interface DoorThreeMakerProps {
  meshRef: React.Ref<Mesh>;
  door: DoorModel;
  height: number;
  color: string;
  onDoorClick?: (ev: ThreeEvent<MouseEvent>) => void;
}

export const DoorThreeMaker = ({
  meshRef,
  door,
  height,
  color,
  onDoorClick,
}: DoorThreeMakerProps): JSX.Element => {
  const THICKNESS = 0.5;
  const ELEVATION = 0;
  const { v1_x, v1_y, v2_x, v2_y } = door;
  const angle = Math.atan2(v1_y - v2_y, v1_x - v2_x) - Math.PI / 2;
  const rot = new Euler(0, 0, angle);

  const pos = midPoint(v1_x, v1_y, v2_x, v2_y).concat(height / 2 + ELEVATION);
  const dist = distance(v1_x, v1_y, v2_x, v2_y);
  return (
    <>
      <CubeMaker
        meshRef={meshRef}
        key={door.name}
        position={pos}
        size={[THICKNESS, dist, height]}
        rot={rot}
        color={color}
        onDoorClick={(ev: ThreeEvent<MouseEvent>) => onDoorClick && onDoorClick(ev)}
      />
    </>
  );
};
