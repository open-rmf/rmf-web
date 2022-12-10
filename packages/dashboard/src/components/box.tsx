import * as THREE from 'three';
import { useRef, useState } from 'react';
import React from 'react';
import { Canvas, MeshProps, useFrame } from '@react-three/fiber';

export interface CubeProps {
  position: number[];
  // thickness, length, height
  rot: THREE.Euler;
  size: number[];
  meshRef?: React.Ref<THREE.Mesh>;
  color?: string;
}

export function Cube({ position, size, rot, color, meshRef }: CubeProps, props: MeshProps) {
  const ref = useRef<THREE.Mesh>(null!);

  return (
    <mesh
      {...props}
      position={[position[0], position[1], position[2]]}
      rotation={rot}
      ref={meshRef || null}
    >
      <boxGeometry args={[size[0], size[1], size[2]]} />
      <meshStandardMaterial color={color || 'blue'} />
    </mesh>
  );
}
