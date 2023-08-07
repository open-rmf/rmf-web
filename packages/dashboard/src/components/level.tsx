import * as THREE from 'three';
import { Level, Lift } from 'api-client';
import { graphToWalls, Wall } from './wall';
import { Door } from './door';
import './temporalStyle.css';

export interface BuildingMapProps {
  level: Level;
  lifts: Lift[];
}

export const BuildingCubes = ({ level, lifts }: BuildingMapProps) => {
  const { doors, elevation } = level;

  return (
    <>
      {level.wall_graph.edges.length > 0 ? (
        <Wall elevation={elevation} opacity={1} walls={level.wall_graph} />
      ) : null}

      {doors.length &&
        doors.map((door, i) => (
          <Door key={i} door={door} opacity={0.1} height={8} elevation={elevation} />
        ))}

      {lifts.length &&
        lifts.map((lift, i) =>
          lift.doors.map((door, i) => (
            <Door key={i} door={door} opacity={0.1} height={8} elevation={elevation} lift={lift} />
          )),
        )}
    </>
  );
};

export const findSceneBoundingBox = (level: Level | undefined): THREE.Box3 | undefined => {
  if (!level) {
    return;
  }
  let minX = Infinity;
  let minY = Infinity;
  let minZ = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  let maxZ = -Infinity;

  const walls = graphToWalls(level.wall_graph, level.elevation);
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

  return new THREE.Box3(new THREE.Vector3(minX, minY, minZ), new THREE.Vector3(maxX, maxY, maxZ));
};
