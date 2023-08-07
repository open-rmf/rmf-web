import { RobotData } from './robots-overlay';
import React from 'react';
import { ThreeEvent, useLoader } from '@react-three/fiber';
import { Box, Html, Line, Text } from '@react-three/drei';

import * as THREE from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader';
import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader';
import { Level } from 'api-client';
import { TrajectoryData } from './trajectories-overlay';
import { Place } from 'react-components';

interface RobotShapeProps {
  robots: RobotData[];
  robotLocations: Record<string, [number, number, number]>;
  level: Level;
  trajectories: TrajectoryData[];
  waypoints: Place[];
  onRobotClick?: (ev: ThreeEvent<MouseEvent>, robot: RobotData) => void;
}

interface TrajectoryComponentProps {
  points: THREE.Vector3[];
  color: string;
}

const TrajectoryComponent: React.FC<TrajectoryComponentProps> = ({ points, color }) => {
  const ref = React.useRef<THREE.Line>();
  return <Line points={points} color={color} linewidth={10} renderOrder={1} />;
};
interface TrajectoryOverlayProps {
  trajectories: TrajectoryData[];
}

interface WaypointProps {
  place: Place;
}

interface PlaneRobotProps {
  position: [number, number];
  color: string;
  opacity: number;
  scale: THREE.Vector3;
  elevation: number;
  rotation: THREE.Euler;
  onRobotClick?: (ev: ThreeEvent<MouseEvent>) => void;
  robot: RobotData;
}

const TrajectoryOverlay: React.FC<TrajectoryOverlayProps> = ({ trajectories }) => {
  return (
    <>
      {trajectories.map((trajData) => (
        <TrajectoryComponent
          key={trajData.trajectory.id}
          points={trajData.trajectory.segments.map((seg) => {
            return new THREE.Vector3(seg.x[0], seg.x[1], 4);
          })}
          color={trajData.color}
        />
      ))}
    </>
  );
};

function PlaneRobot({
  position,
  color,
  opacity,
  scale,
  elevation,
  rotation,
  onRobotClick,
}: PlaneRobotProps) {
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
    <group
      position={[position[0], position[1], positionZ[2]]}
      rotation={rotation}
      scale={scale}
      onClick={onRobotClick}
    >
      <primitive object={object} ref={objectRef} color={color} opacity={opacity} />
      <primitive object={object.clone()} ref={objectRef} />
    </group>
  );
}

const Waypoint = ({ place }: WaypointProps) => {
  const { vertex } = place;
  return (
    <group position={[vertex.x, vertex.y, 0]}>
      <Text position={[0, 0.3, 4.5]} color="orange" fontSize={0.4}>
        {vertex.name}
      </Text>
      <mesh position={[0, 0, 4]} scale={[0.7, 0.7, 0.7]}>
        <boxGeometry args={[0.3, 0.3, 0.3]} />
        <meshStandardMaterial color="yellow" opacity={1} />
      </mesh>
      {/* <Html position={[0, 0.3, 4.5]}>
        <div style={{ color: 'orange', fontSize: 'rem' }}>{vertex.name}</div>
      </Html> */}
    </group>
  );
};

export function RobotShape({
  robots,
  robotLocations,
  level,
  trajectories,
  waypoints,
  onRobotClick,
}: RobotShapeProps) {
  const { elevation } = level;
  return (
    <>
      {waypoints.map((place, index) => (
        <Waypoint key={index} place={place} />
      ))}
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
            onRobotClick={(ev: ThreeEvent<MouseEvent>) => onRobotClick && onRobotClick(ev, robot)}
            robot={robot}
          />
        );
      })}
    </>
  );
}
