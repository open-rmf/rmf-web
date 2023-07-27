import { RobotData } from './robots-overlay';
import React from 'react';
import { Sphere, Plane } from '@react-three/drei';

interface RoboShapeProps {
  robots: RobotData[];
  robotLocations: Record<string, [number, number]>;
}

function PlaneRobot({ shape, rotation, position, color, opacity, index }: any) {
  if (!position) return null;
  const v1 = position[0];
  const v2 = position[1];
  const elevation = 1;
  const height = 8;
  const newPosition = [(v2 + v1) / 2, (v2 + v1) / 2, height / 2 + elevation];

  return (
    <group position={[position[0], position[1], newPosition[2]]} rotation={rotation}>
      <Plane scale={[2, 0.5, 1]} position={[0, 0, 0]} material-color={color} />
      <Plane scale={[1.5, 0.5, 0.5]} position={[-0.25, 0.25, 0.75]} material-color={color} />
      <Sphere args={[0.3, 32, 32]} position={[-0.6, -0.25, 0.75]} />
      <Sphere args={[0.3, 32, 32]} position={[0.6, -0.25, 0.75]} />
    </group>
  );
}

export function RobotShape({ robots, robotLocations }: RoboShapeProps) {
  return (
    <>
      {robots.map((robot, i) => {
        const robotId = `${robot.fleet}/${robot.name}`;
        const robotLocation = robotLocations[robotId];
        return (
          <PlaneRobot
            key={i}
            color={robot.color}
            opacity={1}
            position={[robotLocation[0], robotLocation[1], 0]}
          />
        );
      })}
    </>
  );
}
