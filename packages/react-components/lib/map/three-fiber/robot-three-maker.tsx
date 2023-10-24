import { ThreeEvent } from '@react-three/fiber';
import React from 'react';
import { Euler, Vector3 } from 'three';
import { CircleShape } from '.';
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
      <TextThreeRendering
        position={[position.x, position.y, position.z + 1]}
        text={robot.name}
        data-testid="robot-name"
      />
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
