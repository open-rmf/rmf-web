import { Text } from '@react-three/drei';
import { ThreeEvent, useLoader } from '@react-three/fiber';
import React from 'react';
import { Color, Euler, Texture, TextureLoader, Vector3 } from 'three';
import { CircleShape } from './circle-shape';
import { TextThreeRendering } from './text-maker';

export interface RobotData {
  fleet: string;
  name: string;
  model: string;
  footprint: number;
  scale: number;
  color: string;
  inConflict?: boolean;
  iconPath?: string;
}

interface RobotThreeMakerProps {
  imageUrl?: string;
  robot: RobotData;
  position: Vector3;
  onRobotClick?: (ev: ThreeEvent<MouseEvent>, robot: RobotData) => void;
  rotation: Euler;
  circleSegment: number;
  fontPath?: string;
  robotLabel: boolean;
}

interface RobotImageMakerProps {
  imageUrl: string;
  robot: RobotData;
  position: Vector3;
  onRobotClick?: (ev: ThreeEvent<MouseEvent>, robot: RobotData) => void;
  rotation: Euler;
}

const RobotImageMaker = ({
  imageUrl,
  position,
  rotation,
  onRobotClick,
  robot,
}: RobotImageMakerProps): JSX.Element => {
  const alphaTestThreshold = 0.5;
  const texture: Texture | undefined = useLoader(TextureLoader, imageUrl, undefined, (err) => {
    console.error(`Error loading image from ${imageUrl}:`, err);
  });

  if (!texture) {
    console.error(`Failed to create image texture with ${robot.iconPath}.`);
    return <></>;
  }

  return (
    <>
      <mesh
        position={position}
        rotation={new Euler(0, 0, rotation.z)}
        onClick={(ev: ThreeEvent<MouseEvent>) => onRobotClick && onRobotClick(ev, robot)}
      >
        <planeGeometry
          attach="geometry"
          args={[texture.image.width * robot.scale, texture.image.height * robot.scale]}
        />
        <meshBasicMaterial
          attach="material"
          map={texture}
          color={new Color(robot.color)}
          alphaTest={alphaTestThreshold}
          toneMapped={false}
        />
      </mesh>
    </>
  );
};

export const RobotThreeMaker = ({
  imageUrl,
  robot,
  position,
  onRobotClick,
  rotation,
  circleSegment,
  fontPath,
  robotLabel,
}: RobotThreeMakerProps): JSX.Element => {
  return (
    <>
      {robotLabel && fontPath && fontPath.length > 0 ? (
        <Text
          color="black"
          font={fontPath}
          fontSize={0.5}
          position={[position.x, position.y, position.z + 1]}
        >
          {robot.name}
        </Text>
      ) : robotLabel ? (
        <TextThreeRendering position={[position.x, position.y, position.z + 1]} text={robot.name} />
      ) : null}
      {imageUrl ? (
        <RobotImageMaker
          imageUrl={imageUrl}
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
