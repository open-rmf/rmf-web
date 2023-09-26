import { Circle, Html } from '@react-three/drei';
import React from 'react';

interface ShapeThreeRenderingProps {
  position: [number, number, number];
  color: string;
  text?: string;
  circleShape: boolean;
}

export const ShapeThreeRendering = ({
  position,
  color,
  text,
  circleShape,
}: ShapeThreeRenderingProps): JSX.Element => {
  const HEIGHT = 8;
  const ELEVATION = 0;
  const positionZ = HEIGHT / 2 + ELEVATION;
  const [isHovered, setIsHovered] = React.useState(false);

  const handlePointerOver = () => {
    setIsHovered(true);
  };

  const handlePointerOut = () => {
    setIsHovered(false);
  };

  const scaleFactor = isHovered ? 2 : 1.0;

  return (
    <>
      {circleShape ? (
        <Circle args={[0.3, 64]} position={[position[0], position[1], positionZ]}>
          <meshBasicMaterial color={color} />
        </Circle>
      ) : (
        <group position={position}>
          <mesh
            position={[-1, 0, positionZ]}
            onPointerOver={handlePointerOver}
            onPointerOut={handlePointerOut}
          >
            {text && isHovered && (
              <Html zIndexRange={[0, 0, 1]}>
                {text && (
                  <div
                    style={{
                      backgroundColor: 'rgba(255, 255, 255, 0.8)',
                      padding: '0.2rem 0.5rem',
                      borderRadius: '4px',
                      fontSize: '0.6rem',
                      transform: `scale(${scaleFactor})`,
                      transition: 'transform 0.3s',
                      zIndex: isHovered ? 0.1 : 6,
                    }}
                  >
                    {text}
                  </div>
                )}
              </Html>
            )}
            <boxGeometry args={[0.8, 0.8, 0.8]} />
            <meshStandardMaterial color={color} opacity={1} />
          </mesh>
        </group>
      )}
    </>
  );
};
