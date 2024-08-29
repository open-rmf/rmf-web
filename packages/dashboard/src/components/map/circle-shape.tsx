import { Circle, Line } from '@react-three/drei';
import { MeshProps, ThreeEvent } from '@react-three/fiber';
import { Euler, Vector3 } from 'three';

import { RobotData } from './robot-three-maker';

interface CircleShapeProps extends MeshProps {
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
  onPointerOver,
  onPointerOut,
}: CircleShapeProps): JSX.Element => {
  const SCALED_RADIUS = 0.7;

  const rotatedX = position.x + SCALED_RADIUS * Math.cos(rotation.z);
  const rotatedY = position.y + SCALED_RADIUS * Math.sin(rotation.z);

  return (
    <>
      <Circle
        args={[SCALED_RADIUS, segment]}
        position={position}
        rotation={rotation}
        onClick={onRobotClick}
        onPointerOver={onPointerOver}
        onPointerOut={onPointerOut}
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
