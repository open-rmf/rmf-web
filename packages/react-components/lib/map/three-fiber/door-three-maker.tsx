import React from 'react';
import { CubeMaker } from './cube-maker';
import { Door as DoorModel } from 'rmf-models';
import { Euler } from 'three';

const distance = (v1_x: number, v1_y: number, v2_x: number, v2_y: number) =>
  Math.hypot(v2_x - v1_x, v2_y - v1_y);
const midPoint = (v1_x: number, v1_y: number, v2_x: number, v2_y: number) => [
  (v2_x + v1_x) / 2,
  (v2_y + v1_y) / 2,
];

interface DoorThreeMakerProps {
  meshRef: React.Ref<THREE.Mesh>;
  door: DoorModel;
  height: number;
  color: string;
}

export const DoorThreeMaker = ({
  meshRef,
  door,
  height,
  color,
}: DoorThreeMakerProps): JSX.Element => {
  const THICKNESS = 0.5;
  const ELEVATION = 0;
  const { v1_x, v1_y, v2_x, v2_y } = door;
  const angle = Math.atan2(v1_y - v2_y, v1_x - v2_x) - Math.PI / 2;
  const rot = new Euler(0, 0, angle);

  const pos = midPoint(v1_x, v1_y, v2_x, v2_y).concat(height / 2 + ELEVATION);
  const dist = distance(v1_x, v1_y, v2_x, v2_y);
  return (
    <CubeMaker
      meshRef={meshRef}
      key={door.name}
      position={pos}
      size={[THICKNESS, dist, height]}
      rot={rot}
      color={color}
    />
  );
};
