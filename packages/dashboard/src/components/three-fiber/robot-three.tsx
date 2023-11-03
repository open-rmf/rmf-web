import { ThreeEvent } from '@react-three/fiber';
import React from 'react';
import { RobotThreeMaker, RobotData } from 'react-components';
import { Euler, Vector3 } from 'three';

interface RobotThreeProps {
  robot: RobotData;
  robotLocation: [number, number, number];
  onRobotClick?: (ev: ThreeEvent<MouseEvent>, robot: RobotData) => void;
}

async function exists(url: string) {
  const result = await fetch(url, { method: 'HEAD' });
  return result.ok;
}

export const RobotThree = ({ robot, robotLocation, onRobotClick }: RobotThreeProps) => {
  const STANDAR_Z_POSITION = 5;
  const CIRCLE_SEGMENT = 64;
  const [fontPath, setFontPath] = React.useState<string | undefined>(undefined);

  React.useEffect(() => {
    const newFontPath =
      process.env.PUBLIC_URL && process.env.PUBLIC_URL.length > 0
        ? `${process.env.PUBLIC_URL}/roboto-v18-KFOmCnqEu92Fr1Mu4mxM.woff`
        : '/roboto-v18-KFOmCnqEu92Fr1Mu4mxM.woff';
    (async () => {
      if (await exists(newFontPath)) {
        setFontPath(newFontPath);
      }
    })();
  });

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
        fontPath={fontPath}
      />
    </React.Fragment>
  );
};
