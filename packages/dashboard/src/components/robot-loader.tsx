import { RobotData } from './robots-overlay';
import React from 'react';
import { ThreeEvent, useLoader } from '@react-three/fiber';
import { Line, Circle, Text } from '@react-three/drei';
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
  name: string;
  color: string;
  opacity: number;
  scale: THREE.Vector3;
  elevation: number;
  rotation: THREE.Euler;
  onRobotClick?: (ev: ThreeEvent<MouseEvent>) => void;
  robot: RobotData;
}

const calculateApproximateRadius = (x: number, y: number): number => {
  return Math.sqrt(x * x + y * y);
};

function PlaneRobot({
  position,
  color,
  opacity,
  scale,
  name,
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
  const radius = calculateApproximateRadius(position[0], position[1]);

  const scalingFactor = 0.03;
  const scaledRadius = radius * scalingFactor;
  const rotationZ = rotation.z;

  const rotatedX = position[0] + scaledRadius * Math.cos(rotationZ - Math.PI / 2);
  const rotatedY = position[1] + scaledRadius * Math.sin(rotationZ - Math.PI / 2);
  return (
    // <group
    //   position={[position[0], position[1], zPosition]}
    //   rotation={rotation}
    //   scale={scale}
    //   onClick={onRobotClick}
    // >
    <>
      <Text color="black" fontSize={0.5} position={[position[0], position[1], zPosition + 1]}>
        {name}
      </Text>
      <Circle
        args={[scaledRadius, 64]}
        position={[position[0], position[1], zPosition]}
        rotation={rotation}
        onClick={onRobotClick}
      >
        <meshBasicMaterial color={color} />
      </Circle>
      <Line
        points={[position[0], position[1], zPosition, rotatedX, rotatedY, zPosition]}
        color="black"
        linewidth={2}
      />
      {/* </group> */}
    </>
  );
}

export function ObjectLoader({ robots, robotLocations, level, onRobotClick }: ObjectLoaderProps) {
  const { elevation } = level;
  return (
    <>
      {robots.map((robot, i) => {
        console.log(robot);
        const robotId = `${robot.fleet}/${robot.name}`;
        const robotLocation = robotLocations[robotId];
        const scale = new THREE.Vector3(0.007, 0.007, 0.007);
        return (
          <PlaneRobot
            key={i}
            name={robot.name}
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
