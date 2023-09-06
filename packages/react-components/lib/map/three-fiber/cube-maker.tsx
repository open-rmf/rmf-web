import { Euler, Mesh } from 'three';
import React from 'react';
import { MeshProps } from '@react-three/fiber';

export interface CubeProps {
  position: number[];
  rot: Euler;
  size: number[];
  meshRef?: React.Ref<Mesh>;
  color?: string;
}

export const CubeMaker = (
  { position, size, rot, color, meshRef }: CubeProps,
  props: MeshProps,
): JSX.Element => {
  return (
    <mesh
      {...props}
      position={[position[0], position[1], position[2]]}
      rotation={rot}
      scale={[size[0], size[1], size[2]]}
      ref={meshRef || null}
    >
      <planeGeometry />
      <meshStandardMaterial color={color || 'black'} opacity={0.6} transparent />
    </mesh>
  );
};
