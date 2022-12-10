import * as THREE from 'three';
import React, { useRef, useState } from 'react';
import { Canvas, useFrame, MeshProps } from '@react-three/fiber';
import { Graph } from 'api-client';
import { GraphNode } from 'rmf-models';
import { Level as BuildingLevel } from 'api-client';
import { Wall } from './wall';
// import { DoorVisual } from './door';
import { Cube } from './box';

export interface LevelProps {
  level: BuildingLevel;
  show: boolean;
}

export function Level({ level, show }: LevelProps, props: MeshProps) {
  //   const ref = useRef<THREE.Mesh>(null!);
  const { doors } = level;
  const rot = new THREE.Euler(0, 0, 0);
  return (
    <>
      {/* <Cube           rot={rot}
          size={[1, 1, 10]}
          position={[0, 0, 0]} /> */}
      {level.wall_graph.edges.length > 0 ? (
        <Wall opacity={Number(show)} walls={level.wall_graph}></Wall>
      ) : null}
      {/* {level.doors.map((door, i) => (
        <DoorVisual opacity={Number(selected)} door={door} />
      ))} */}
    </>
  );
}
