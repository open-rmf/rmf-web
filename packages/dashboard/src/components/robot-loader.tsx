import { RobotData } from './robots-overlay';
import React from 'react';
import { ThreeEvent, useLoader } from '@react-three/fiber';

import * as THREE from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader';
import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader';
import { Level } from 'api-client';

interface ObjectLoaderProps {
  robots: RobotData[];
  robotLocations: Record<string, [number, number, number]>;
  level: Level;
  onRobotClick?: (ev: ThreeEvent<MouseEvent>, robot: RobotData) => void;
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

function PlaneRobot({
  position,
  color,
  opacity,
  scale,
  elevation,
  rotation,
  onRobotClick,
}: PlaneRobotProps) {
  const height = 8;
  const zPosition = height / 2 + 0;

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
      position={[position[0], position[1], zPosition]}
      rotation={rotation}
      scale={scale}
      onClick={onRobotClick}
    >
      <primitive object={object} ref={objectRef} color={color} opacity={opacity} />
      <primitive object={object.clone()} ref={objectRef} />
    </group>
  );
}

export function ObjectLoader({ robots, robotLocations, level, onRobotClick }: ObjectLoaderProps) {
  const { elevation } = level;
  return (
    <>
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
