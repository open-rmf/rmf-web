import { MeshProps, ThreeEvent } from '@react-three/fiber';
import React from 'react';
import { Euler, Mesh } from 'three';

export interface CubeProps {
  position: number[];
  rot: Euler;
  size: number[];
  meshRef?: React.Ref<Mesh>;
  color?: string;
  onDoorClick?: (ev: ThreeEvent<MouseEvent>) => void;
}

export const CubeMaker = (
  { position, size, rot, color, meshRef, onDoorClick }: CubeProps,
  props: MeshProps,
): JSX.Element => {
  return (
    <mesh
      {...props}
      position={[position[0], position[1], position[2]]}
      rotation={rot}
      scale={[size[0], size[1], size[2]]}
      ref={meshRef}
      onClick={(ev: ThreeEvent<MouseEvent>) => onDoorClick && onDoorClick(ev)}
    >
      <planeGeometry />
      <meshStandardMaterial color={color || 'black'} opacity={0.6} transparent />
    </mesh>
  );
};
