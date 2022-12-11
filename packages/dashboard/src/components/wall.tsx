import * as THREE from 'three';
import { useRef } from 'react';
import { MeshProps } from '@react-three/fiber';
import { Graph } from 'api-client';
import { GraphNode } from 'rmf-models';
import { Cube } from './cube';

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

function distance(v1: GraphNode, v2: GraphNode) {
  return Math.hypot(v2.x - v1.x, v2.y - v1.y);
}

function midPoint(v1: GraphNode, v2: GraphNode) {
  return [(v2.x + v1.x) / 2, (v2.y + v1.y) / 2];
}

function graphToWalls(graph: Graph, elevation: number) {
  const walls = [] as WallSegment[];
  const { edges, vertices } = graph;
  edges.map((edge) => {
    const v1 = vertices[edge.v1_idx];
    const v2 = vertices[edge.v2_idx];

    const depth = distance(v1, v2);
    const midpoint = midPoint(v1, v2);
    const width = 0.5;
    const heigt = 8;
    const position = midpoint.concat(heigt / 2 + elevation);
    const v = new THREE.Vector3(v1.x - v2.x, 0, v1.y - v2.y);
    v.normalize();
    const rot = new THREE.Euler(0, 0, v.angleTo(new THREE.Vector3(0, 0, 1)));
    walls.push({
      position: position,
      width: width,
      height: heigt,
      depth: depth,
      rot: rot,
    } as WallSegment);
  });
  return walls;
}

export function Wall({ walls, elevation, opacity }: WallProps, props: MeshProps) {
  const ref = useRef<THREE.Mesh>(null!);
  const walls_ = graphToWalls(walls, elevation);
  return (
    <>
      {walls_.map((wall, i) => (
        <Cube
          key={i}
          rot={walls_[i].rot}
          size={[walls_[i].width, walls_[i].depth, walls_[i].height]}
          position={[walls_[i].position[0], walls_[i].position[1], walls_[i].position[2]]}
        ></Cube>
      ))}
    </>
  );
}
