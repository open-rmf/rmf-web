import { RobotData } from './robots-overlay';
import React from 'react';
import { useLoader } from '@react-three/fiber';
import { Line } from '@react-three/drei';

import * as THREE from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader';
import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader';
import { Level } from 'api-client';
import { TrajectoryData } from './trajectories-overlay';

interface RobotShapeProps {
  robots: RobotData[];
  robotLocations: Record<string, [number, number, number]>;
  level: Level;
  trajectories: TrajectoryData[];
}

interface TrajectoryComponentProps {
  points: THREE.Vector3[];
  color: string;
}

const TrajectoryComponent: React.FC<TrajectoryComponentProps> = ({ points, color }) => {
  const ref = React.useRef<THREE.Line>();
  return <Line points={points} color={color} linewidth={2} />;
};
interface TrajectoryOverlayProps {
  trajectories: TrajectoryData[];
}

const TrajectoryOverlay: React.FC<TrajectoryOverlayProps> = ({ trajectories }) => {
  return (
    <>
      {trajectories.map((trajData) => (
        <TrajectoryComponent
          key={trajData.trajectory.id}
          points={trajData.trajectory.segments.map(
            (seg) => new THREE.Vector3(seg.x[0], seg.x[1], seg.x[2]),
          )}
          color={trajData.color}
        />
      ))}
    </>
  );
};

function PlaneRobot({ position, color, opacity, scale, elevation, rotation }: any) {
  const v1 = position[0];
  const v2 = position[1];
  const height = 8;
  const positionZ = [(v2 + v1) / 2, (v2 + v1) / 2, height / 2 + elevation];

  const objPath = '/Hatchback/meshes/hatchback.obj';
  const mtlPath = '/Hatchback/meshes/hatchback.mtl';
  const objectRef = React.useRef<THREE.Object3D>(null);

  const materials = useLoader(MTLLoader, mtlPath);
  const object = useLoader(OBJLoader, objPath, (loader) => {
    materials.preload();
    loader.setMaterials(materials);
  });
  return (
    <group position={[position[0], position[1], positionZ[2]]} rotation={rotation} scale={scale}>
      <primitive object={object} ref={objectRef} />
      <primitive object={object.clone()} ref={objectRef} />
    </group>
  );
}

export function RobotShape({ robots, robotLocations, level, trajectories }: RobotShapeProps) {
  const { elevation } = level;
  return (
    <>
      <TrajectoryOverlay trajectories={trajectories} />
      {robots.map((robot, i) => {
        const robotId = `${robot.fleet}/${robot.name}`;
        const robotLocation = robotLocations[robotId];
        const scale = new THREE.Vector3(0.007, 0.007, 0.007);
        return (
          <PlaneRobot
            key={i}
            color={robot.color}
            opacity={1}
            position={[robotLocation[0], robotLocation[1]]}
            rotation={new THREE.Euler(0, 0, robotLocation[2] + Math.PI / 2)}
            scale={scale}
            elevation={elevation}
          />
        );
      })}
    </>
  );
}
