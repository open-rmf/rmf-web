import { Text } from '@react-three/drei';

interface ThreeDObjectProps {
  position: [number, number, number];
  color: string;
  fontSize?: number;
  scale: [number, number, number];
  text?: string;
  elevation: number;
}

export const ThreeDObject = ({
  position,
  color,
  fontSize,
  scale,
  text,
  elevation,
}: ThreeDObjectProps) => {
  const height = 8;
  const zPosition = height / 2 + elevation;
  return (
    <group position={position}>
      {text && (
        <Text position={[0, 0.7, zPosition]} color="orange" fontSize={fontSize}>
          {text}
        </Text>
      )}
      <mesh position={[0, 0, zPosition]} scale={scale}>
        <boxGeometry args={[0.3, 0.3, 0.3]} />
        <meshStandardMaterial color={color} opacity={1} />
      </mesh>
    </group>
  );
};
