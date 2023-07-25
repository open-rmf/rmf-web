import * as THREE from 'three';
import { useRef } from 'react';
import { MeshProps } from '@react-three/fiber';
import { Graph } from 'api-client';
import { GraphNode } from 'rmf-models';
import { Cube } from './cube';
import React from 'react';

interface WallProps {
  walls: Graph;
  opacity: number;
  elevation: number;
}

interface WallSegment {
  position: number[];
  width: number;
  height: number;
  depth: number;
  rot: THREE.Euler;
}

const distance = (v1: GraphNode, v2: GraphNode) => Math.hypot(v2.x - v1.x, v2.y - v1.y);
const midPoint = (v1: GraphNode, v2: GraphNode) => [(v2.x + v1.x) / 2, (v2.y + v1.y) / 2];

export const graphToWalls = (graph: Graph, elevation: number) => {
  const walls = [] as WallSegment[];
  const { edges, vertices } = graph;

  edges.map((edge) => {
    const v1 = vertices[edge.v1_idx];
    const v2 = vertices[edge.v2_idx];

    const depth = distance(v1, v2);
    const midpoint = midPoint(v1, v2);
    const width = 0.3;
    const height = 8;
    const position = midpoint.concat(height / 2 + elevation);

    const angle = Math.atan2(v1.y - v2.y, v1.x - v2.x) - Math.PI / 2;
    const rot = new THREE.Euler(0, 0, angle);

    return walls.push({
      position,
      width,
      height,
      depth,
      rot,
    });
  });

  return walls;
};

export function Wall({ walls, elevation, opacity }: WallProps, props: MeshProps) {
  const ref = useRef<THREE.Mesh>(null!);
  const walls_ = graphToWalls(walls, elevation);

  return (
    <>
      {walls_.map((wall, i) => {
        return (
          <Cube
            key={i}
            rot={wall.rot}
            size={[wall.width, wall.depth, wall.height]}
            position={[wall.position[0], wall.position[1], wall.position[2]]}
          />
        );
      })}
    </>
  );
}
