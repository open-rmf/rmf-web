import { useGLTF, PerspectiveCamera } from '@react-three/drei';
import { GLTF } from 'three-stdlib';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader';
import { useLoader } from '@react-three/fiber';
import { Svg } from '@react-three/drei';
import { RobotData } from './robots-overlay';
import * as THREE from 'three';

interface RoboShapeProps {
  robots: RobotData[];
  robotLocations: Record<string, [number, number]>;
}

function Shape({ shape, rotation, position, color, opacity, index }: any) {
  if (!position) return null;
  const v1 = position[0];
  const v2 = position[1];
  const elevation = 1;
  const height = 8;
  const newPosition = [(v2 + v1) / 2, (v2 + v1) / 2].concat(height / 2 + elevation);

  return (
    // <mesh rotation={rotation} position={[position[0], position[1], newPosition[2]]}>
    //     <meshStandardMaterial color={color} opacity={opacity} />
    //     <shapeGeometry args={[shape]} />
    // </mesh>
    <mesh
      position={[position[0], position[1], newPosition[2]]}
      rotation={rotation}
      // scale={[size[0], size[1], size[2]]}
      ref={null}
    >
      <planeGeometry />
      {/* <boxGeometry /> */}
      <meshStandardMaterial color={color || 'blue'} opacity={0.7} transparent />
    </mesh>
  );
}

export function RobotShape({ robots, robotLocations }: RoboShapeProps) {
  return (
    <>
      {robots.map((robot, i) => {
        const robotId = `${robot.fleet}/${robot.name}`;
        const robotLocation = robotLocations[robotId];
        return (
          <Shape
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
