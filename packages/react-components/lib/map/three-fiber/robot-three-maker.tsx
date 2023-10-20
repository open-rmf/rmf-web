import { Circle, Line, Text } from '@react-three/drei';
import { ThreeEvent } from '@react-three/fiber';
import React from 'react';
import { Euler, Vector3 } from 'three';
import { TextThreeRendering } from './text-maker';

export interface RobotData {
  fleet: string;
  name: string;
  model: string;
  footprint: number;
  color: string;
  inConflict?: boolean;
  iconPath?: string;
}

interface CircleShapeProps {
  position: Vector3;
  rotation: Euler;
  onRobotClick?: (ev: ThreeEvent<MouseEvent>) => void;
  robot: RobotData;
  segment: number;
}

interface RobotThreeMakerProps {
  robot: RobotData;
  position: Vector3;
  onRobotClick?: (ev: ThreeEvent<MouseEvent>, robot: RobotData) => void;
  rotation: Euler;
  circleSegment: number;
}

const CircleShape = ({
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

export const RobotThreeMaker = ({
  robot,
  position,
  onRobotClick,
  rotation,
  circleSegment,
}: RobotThreeMakerProps): JSX.Element => {
  return (
    <>
      <TextThreeRendering position={[position.x, position.y, position.z + 1]} text={robot.name} />
      <CircleShape
        position={position}
        rotation={rotation}
        onRobotClick={(ev: ThreeEvent<MouseEvent>) => onRobotClick && onRobotClick(ev, robot)}
        robot={robot}
        segment={circleSegment}
      />
    </>
  );
};
