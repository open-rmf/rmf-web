import type { AffineImage, Door, Location2D } from 'api-client';
import { Graph, Level } from 'api-client';
import { Door as RmfDoor, GraphNode } from 'rmf-models/ros/rmf_building_map_msgs/msg';
import { Box3, Euler, Vector3 } from 'three';

import { fromRmfCoords, fromRmfYaw } from '../utils';

export function loadAffineImage(image: AffineImage): Promise<HTMLImageElement> {
  const imageElement = new Image();
  const imageUrl = image.data;
  imageElement.src = imageUrl;

  return new Promise((res) => {
    const listener = () => {
      imageElement.removeEventListener('load', listener);
      res(imageElement);
    };
    imageElement.addEventListener('load', listener);
  });
}

export function getRmfTransform(location: Location2D): string {
  const pos = fromRmfCoords([location.x, location.y]);
  const yaw = (fromRmfYaw(location.yaw) / Math.PI) * 180;
  return `translate(${pos[0]} ${pos[1]}) rotate(${yaw})`;
}

export function getDoorCenter(door: Door): [x: number, y: number] {
  const v1 = [door.v1_x, door.v1_y];
  const v2 = [door.v2_x, door.v2_y];
  switch (door.door_type) {
    case RmfDoor.DOOR_TYPE_SINGLE_SLIDING:
    case RmfDoor.DOOR_TYPE_SINGLE_SWING:
    case RmfDoor.DOOR_TYPE_SINGLE_TELESCOPE:
    case RmfDoor.DOOR_TYPE_DOUBLE_SLIDING:
    case RmfDoor.DOOR_TYPE_DOUBLE_SWING:
    case RmfDoor.DOOR_TYPE_DOUBLE_TELESCOPE:
      return [(v1[0] + v2[0]) / 2, (v2[1] + v1[1]) / 2];
    default:
      throw new Error('unknown door type');
  }
}

interface WallSegment {
  position: number[];
  width: number;
  height: number;
  depth: number;
  rot: Euler;
}

const distance = (v1: GraphNode, v2: GraphNode) => Math.hypot(v2.x - v1.x, v2.y - v1.y);
const midPoint = (v1: GraphNode, v2: GraphNode) => [(v2.x + v1.x) / 2, (v2.y + v1.y) / 2];

export const graphToWalls = (graph: Graph): WallSegment[] => {
  const walls = [] as WallSegment[];
  const { edges, vertices } = graph;

  edges.map((edge) => {
    const v1 = vertices[edge.v1_idx];
    const v2 = vertices[edge.v2_idx];

    const depth = distance(v1, v2);
    const midpoint = midPoint(v1, v2);
    const width = 0.3;
    const height = 8;
    const position = midpoint.concat(height / 2 + 0);

    const angle = Math.atan2(v1.y - v2.y, v1.x - v2.x) - Math.PI / 2;
    const rot = new Euler(0, 0, angle);

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

export const findSceneBoundingBoxFromThreeFiber = (level: Level | undefined): Box3 | undefined => {
  if (!level) {
    return;
  }
  let minX = Infinity;
  let minY = Infinity;
  let minZ = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  let maxZ = -Infinity;

  const walls = graphToWalls(level.wall_graph);
  walls.forEach((wall) => {
    const [x, y, z] = wall.position;
    const width = wall.width;
    const height = wall.height;

    minX = Math.min(minX, x - width / 2);
    minY = Math.min(minY, y - width / 2);
    minZ = Math.min(minZ, z - height / 2);
    maxX = Math.max(maxX, x + width / 2);
    maxY = Math.max(maxY, y + width / 2);
    maxZ = Math.max(maxZ, z + height / 2);
  });

  return new Box3(new Vector3(minX, minY, minZ), new Vector3(maxX, maxY, maxZ));
};
