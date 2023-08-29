import { Circle, Text } from '@react-three/drei';

interface ThreeDObjectProps {
  position: [number, number, number];
  color: string;
  text?: string;
  elevation: number;
  circleShape: boolean;
}

export const ThreeDObject = ({
  position,
  color,
  text,
  elevation,
  circleShape,
}: ThreeDObjectProps) => {
  const height = 8;
  const zPosition = height / 2 + elevation;
  return (
    <>
      {circleShape ? (
        <Circle args={[0.3, 64]} position={[position[0], position[1], zPosition]}>
          <meshBasicMaterial color={color} />
        </Circle>
      ) : (
        <group position={position}>
          {text && (
            <Text position={[0, 0.7, zPosition]} color="orange" fontSize={0.6}>
              {text}
            </Text>
          )}
          <mesh position={[0, 0, zPosition]} scale={[0.7, 0.7, 0.7]}>
            <boxGeometry args={[0.3, 0.3, 0.3]} />
            <meshStandardMaterial color={color} opacity={1} />
          </mesh>
        </group>
      )}
    </>
  );
};
