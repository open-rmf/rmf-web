import * as THREE from 'three';
import React from 'react';
import { MeshProps } from '@react-three/fiber';

export interface CubeProps {
  position: number[];
  rot: THREE.Euler;
  size: number[];
  meshRef?: React.Ref<THREE.Mesh>;
  color?: string;
}

export function Cube({ position, size, rot, color, meshRef }: CubeProps, props: MeshProps) {
  return (
    <mesh
      {...props}
      position={[position[0], position[1], position[2]]}
      rotation={rot}
      scale={[size[0], size[1], size[2]]}
      ref={meshRef || null}
    >
      <planeGeometry />
      {/* <boxGeometry /> */}
      <meshStandardMaterial color={color || 'blue'} opacity={0.2} transparent />
    </mesh>
  );
}
