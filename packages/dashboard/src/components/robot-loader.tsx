import { RobotData } from './robots-overlay';
import React from 'react';
import { useLoader } from '@react-three/fiber';

import * as THREE from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader';
import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader';

interface RobotShapeProps {
  robots: RobotData[];
  robotLocations: Record<string, [number, number]>;
}

function PlaneRobot({ shape, position, color, opacity, index, scale }: any) {
  const [x, y] = position;
  const angle = Math.atan2(y, x);

  const rotation = new THREE.Euler(0, 0, angle);

  const v1 = position[0];
  const v2 = position[1];
  const elevation = 1;
  const height = 8;
  const newPosition = [(v2 + v1) / 2, (v2 + v1) / 2, height / 2 + elevation];

  const objPath = '/Hatchback/meshes/hatchback.obj';
  const mtlPath = '/Hatchback/meshes/hatchback.mtl';

  const materials = useLoader(MTLLoader, mtlPath);
  const object = useLoader(OBJLoader, objPath, (loader) => {
    materials.preload();
    loader.setMaterials(materials);
  });
  return (
    <group position={[position[0], position[1], newPosition[2]]} rotation={rotation} scale={scale}>
      <primitive object={object} />
      <primitive object={object.clone()} />
    </group>
  );
}

export function RobotShape({ robots, robotLocations }: RobotShapeProps) {
  return (
    <>
      {robots.map((robot, i) => {
        const robotId = `${robot.fleet}/${robot.name}`;
        const robotLocation = robotLocations[robotId];
        const scale = new THREE.Vector3(0.01, 0.01, 0.01);
        return (
          <PlaneRobot
            key={i}
            color={robot.color}
            opacity={1}
            position={[robotLocation[0], robotLocation[1], 0]}
            scale={scale}
          />
        );
      })}
    </>
  );
}
