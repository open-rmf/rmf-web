import React from 'react';
import { Cube } from './cube';
import { Door as DoorModel } from 'rmf-models';
import { Vector3, Euler } from 'three';

function distance(v1_x: number, v1_y: number, v2_x: number, v2_y: number) {
  return Math.hypot(v2_x - v1_x, v2_y - v1_y);
}

function midPoint(v1_x: number, v1_y: number, v2_x: number, v2_y: number) {
  return [(v2_x + v1_x) / 2, (v2_y + v1_y) / 2];
}

interface SingleDoorProps {
  meshRef: React.Ref<THREE.Mesh>;
  door: DoorModel;
  height: number;
  color: string;
}

export const DoorMakerThree = ({ meshRef, door, height, color }: SingleDoorProps) => {
  const { v1_x, v1_y, v2_x, v2_y } = door;
  const thickness = 0.5;
  const v = new Vector3(v1_x - v2_x, 0, v1_y - v2_y);
  v.normalize();
  const angle = Math.atan2(v1_y - v2_y, v1_x - v2_x) - Math.PI / 2;
  const rot = new Euler(0, 0, angle);

  const pos = midPoint(v1_x, v1_y, v2_x, v2_y).concat(height / 2 + 0);
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
};
