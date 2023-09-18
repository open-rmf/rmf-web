import { Circle, Line, Text } from '@react-three/drei';
import { ThreeEvent, useLoader } from '@react-three/fiber';
import React from 'react';
import { Euler, Vector3 } from 'three';
import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader';

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

interface ObjectLoaderProps {
  position: Vector3;
  rotation: THREE.Euler;
  onRobotClick?: (ev: ThreeEvent<MouseEvent>) => void;
  robot: RobotData;
}

interface RobotThreeMakerProps {
  robot: RobotData;
  position: Vector3;
  onRobotClick?: (ev: ThreeEvent<MouseEvent>, robot: RobotData) => void;
  objectModel?: string;
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
  const SCALED_RADIUS = 1;

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

export const RobotThreeMaker = ({
  robot,
  position,
  onRobotClick,
  objectModel, //Object model should be some path of the object to be rendered.
  rotation,
  circleSegment,
}: RobotThreeMakerProps): JSX.Element => {
  return (
    <>
      <Text color="black" fontSize={0.5} position={[position.x, position.y, position.z + 1]}>
        {robot.name}
      </Text>

      {objectModel ? (
        <ObjectLoader
          position={position}
          rotation={rotation}
          onRobotClick={(ev: ThreeEvent<MouseEvent>) => onRobotClick && onRobotClick(ev, robot)}
          robot={robot}
        />
      ) : (
        <CircleShape
          position={position}
          rotation={rotation}
          onRobotClick={(ev: ThreeEvent<MouseEvent>) => onRobotClick && onRobotClick(ev, robot)}
          robot={robot}
          segment={circleSegment}
        />
      )}
    </>
  );
};
