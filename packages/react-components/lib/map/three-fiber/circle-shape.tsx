import { Circle, Line } from '@react-three/drei';
import { ThreeEvent } from '@react-three/fiber';
import React from 'react';
import { Euler, Vector3 } from 'three';
import { RobotData } from './robot-three-maker';

interface CircleShapeProps {
  position: Vector3;
  rotation: Euler;
  onRobotClick?: (ev: ThreeEvent<MouseEvent>) => void;
  robot: RobotData;
  segment: number;
}

export const CircleShape = ({
  position,
  rotation,
  onRobotClick,
  robot,
  segment,
}: CircleShapeProps): JSX.Element => {
  const SCALED_RADIUS = 0.7;

  const rotatedX = position.x + SCALED_RADIUS * Math.cos(rotation.z - Math.PI / 2);
  const rotatedY = position.y + SCALED_RADIUS * Math.sin(rotation.z - Math.PI / 2);

  return (
    <>
      <Circle
        args={[SCALED_RADIUS, segment]}
        position={position}
        rotation={rotation}
        onClick={onRobotClick}
      >
        <meshBasicMaterial color={robot.color} />
      </Circle>
      <Line
        points={[position.x, position.y, position.z, rotatedX, rotatedY, position.z]}
        color="black"
        linewidth={2}
      />
    </>
  );
};
