import { ThreeEvent } from '@react-three/fiber';
import React from 'react';
import { RobotThreeMaker, RobotData } from 'react-components';
import { Euler, Vector3 } from 'three';

interface RobotThreeProps {
  robot: RobotData;
  robotLocation: [number, number, number];
  onRobotClick?: (ev: ThreeEvent<MouseEvent>, robot: RobotData) => void;
}

export const RobotThree = ({ robot, robotLocation, onRobotClick }: RobotThreeProps) => {
  const STANDAR_Z_POSITION = 5;
  const CIRCLE_SEGMENT = 64;

  const robotId = `${robot.fleet}/${robot.name}`;
  const rotationZ = robotLocation[2] - Math.PI;

  const position = new Vector3(robotLocation[0], robotLocation[1], STANDAR_Z_POSITION);

  return (
    <React.Fragment key={robotId}>
      <RobotThreeMaker
        robot={robot}
        imageUrl={robot.iconPath}
        position={position}
        onRobotClick={onRobotClick}
        rotation={new Euler(0, 0, rotationZ)}
        circleSegment={CIRCLE_SEGMENT}
      />
    </React.Fragment>
  );
};
