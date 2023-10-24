import { Text } from '@react-three/drei';
import { ThreeEvent } from '@react-three/fiber';
import React from 'react';
import { Euler, Vector3 } from 'three';
import { CircleShape } from '.';

export interface RobotData {
  fleet: string;
  name: string;
  model: string;
  footprint: number;
  color: string;
  inConflict?: boolean;
  iconPath?: string;
}

interface RobotThreeMakerProps {
  robot: RobotData;
  position: Vector3;
  onRobotClick?: (ev: ThreeEvent<MouseEvent>, robot: RobotData) => void;
  rotation: Euler;
  circleSegment: number;
}

export const RobotThreeMaker = ({
  robot,
  position,
  onRobotClick,
  rotation,
  circleSegment,
}: RobotThreeMakerProps): JSX.Element => {
  return (
    <>
      <Text
        color="black"
        fontSize={0.5}
        position={[position.x, position.y, position.z + 1]}
        data-testid="robot-name"
      >
        {robot.name}
      </Text>
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
