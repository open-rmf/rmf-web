import { Circle, Line, Text } from '@react-three/drei';
import { ThreeEvent, useLoader } from '@react-three/fiber';
import React from 'react';
import { Euler, Vector3 } from 'three';
import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader';
import { RobotData } from './robots-overlay';

interface CircleShapeProps {
  position: [number, number, number];
  rotation: THREE.Euler;
  onRobotClick?: (ev: ThreeEvent<MouseEvent>) => void;
  robot: RobotData;
  segment: number;
  scalingFactor: number;
}

interface ObjectLoaderProps {
  position: [number, number, number];
  rotation: THREE.Euler;
  onRobotClick?: (ev: ThreeEvent<MouseEvent>) => void;
  robot: RobotData;
}

interface RobotLoaderProps {
  robots: RobotData[];
  robotLocations: Record<string, [number, number, number]>;
  onRobotClick?: (ev: ThreeEvent<MouseEvent>, robot: RobotData) => void;
  objectModel?: string;
}

const calculateApproximateRadius = (x: number, y: number): number => Math.sqrt(x * x + y * y);

const CircleShape = ({
  position,
  rotation,
  onRobotClick,
  robot,
  segment,
  scalingFactor,
}: CircleShapeProps): JSX.Element => {
  const radius = calculateApproximateRadius(position[0], position[1]);
  const scaledRadius = radius * scalingFactor;
  const rotationZ = rotation.z;

  const rotatedX = position[0] + scaledRadius * Math.cos(rotationZ - Math.PI / 2);
  const rotatedY = position[1] + scaledRadius * Math.sin(rotationZ - Math.PI / 2);

  return (
    <>
      <Circle
        args={[scaledRadius, segment]}
        position={position}
        rotation={rotation}
        onClick={onRobotClick}
      >
        <meshBasicMaterial color={robot.color} />
      </Circle>
      <Line
        points={[position[0], position[1], position[2], rotatedX, rotatedY, position[2]]}
        color="black"
        linewidth={2}
      />
    </>
  );
};

const ObjectLoader = ({
  position,
  rotation,
  onRobotClick,
  robot,
}: ObjectLoaderProps): JSX.Element => {
  const objPath = '/Hatchback/meshes/hatchback.obj';
  const mtlPath = '/Hatchback/meshes/hatchback.mtl';
  const objectRef = React.useRef<THREE.Object3D>(null);
  const scale = new Vector3(0.007, 0.007, 0.007);

  const materials = useLoader(MTLLoader, mtlPath);
  const object = useLoader(OBJLoader, objPath, (loader) => {
    materials.preload();
    loader.setMaterials(materials);
  });
  return (
    <group position={position} rotation={rotation} scale={scale} onClick={onRobotClick}>
      <primitive object={object} ref={objectRef} color={robot.color} opacity={1} />
      <primitive object={object.clone()} ref={objectRef} />
    </group>
  );
};

export const RobotLoader = ({
  robots,
  robotLocations,
  onRobotClick,
  objectModel, //Object model should be some path of the object to be rendered.
}: RobotLoaderProps) => {
  const STANDAR_Z_POSITION = 4;
  const CIRCLE_SEGMENT = 64;
  const SCALING_FACTOR = 0.03;

  return (
    <>
      {robots.map((robot) => {
        const robotId = `${robot.fleet}/${robot.name}`;
        const robotLocation = robotLocations[robotId];
        const rotationZ = robotLocation[2] + Math.PI / 2;

        return (
          <React.Fragment key={robotId}>
            <Text
              color="black"
              fontSize={0.5}
              position={[robotLocation[0], robotLocation[1], STANDAR_Z_POSITION + 1]}
            >
              {robot.name}
            </Text>

            {objectModel ? (
              <ObjectLoader
                position={[robotLocation[0], robotLocation[1], STANDAR_Z_POSITION]}
                rotation={new Euler(0, 0, rotationZ)}
                onRobotClick={(ev: ThreeEvent<MouseEvent>) =>
                  onRobotClick && onRobotClick(ev, robot)
                }
                robot={robot}
              />
            ) : (
              <CircleShape
                position={[robotLocation[0], robotLocation[1], STANDAR_Z_POSITION]}
                rotation={new Euler(0, 0, rotationZ)}
                onRobotClick={(ev: ThreeEvent<MouseEvent>) =>
                  onRobotClick && onRobotClick(ev, robot)
                }
                robot={robot}
                segment={CIRCLE_SEGMENT}
                scalingFactor={SCALING_FACTOR}
              />
            )}
          </React.Fragment>
        );
      })}
    </>
  );
};
