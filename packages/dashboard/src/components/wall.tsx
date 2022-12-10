import * as THREE from 'three';
import React, { useRef, useState } from 'react';
import { Canvas, useFrame, MeshProps } from '@react-three/fiber';
import { Graph } from 'api-client';
import { GraphNode } from 'rmf-models';
import { Cube } from './box';
import { LineSegments } from 'three';
export interface WallProps {
  walls: Graph;
  opacity: number;
}

function distance(v1: GraphNode, v2: GraphNode) {
  return Math.hypot(v2.x - v1.x, v2.y - v1.y);
}

function midPoint(v1: GraphNode, v2: GraphNode) {
  return [(v2.x + v1.x) / 2, (v2.y + v1.y) / 2];
}

interface WallSegment {
  position: number[];
  width: number;
  height: number;
  depth: number;
  rot: THREE.Euler;
}

function graphToWalls(graph: Graph) {
  const walls_ = [] as WallSegment[];
  let minx = 999;
  let miny = 999;
  for (const edge of graph.edges) {
    const v1 = graph.vertices[edge.v1_idx];
    const v2 = graph.vertices[edge.v2_idx];
    if (v1.x < minx) {
      minx = v1.x;
    }
    if (v1.y < miny) {
      miny = v1.y;
    }

    const depth = distance(v1, v2);
    const midpoint = midPoint(v1, v2);
    const width = 0.5;
    const heigt = 8;
    const position = midpoint.concat(heigt / 2);
    const v = new THREE.Vector3(v1.x - v2.x, 0, v1.y - v2.y);
    v.normalize();
    const b = v.angleTo(new THREE.Vector3(0, 0, 1));
    const rot = new THREE.Euler(0, 0, b);
    walls_.push({
      position: position,
      width: width,
      height: heigt,
      depth: depth,
      rot: rot,
    } as WallSegment);
  }
  return { walls_, minx, miny };
}

interface WallsProps {
  wallSegments: WallSegment[];
}

export function Wall({ walls }: WallProps, props: MeshProps) {
  //   const ref = useRef<THREE.Mesh>(null!);
  const { walls_ } = graphToWalls(walls);
  const edges = walls.edges;
  const verts = walls.vertices;
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
