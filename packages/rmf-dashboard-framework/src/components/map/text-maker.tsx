import { Html } from '@react-three/drei';
import React from 'react';

interface TextThreeRenderingProps {
  position: [number, number, number];
  text?: string;
}

export const TextThreeRendering = ({ position, text }: TextThreeRenderingProps): JSX.Element => {
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
  const positionX = text && text.length > 5 ? -2 : -1;
  return (
    <>
      <mesh position={position}>
        <mesh position={[positionX, 0, positionZ]}>
          {text && (
            <Html zIndexRange={[0, 0, 1]}>
              {text && (
                <div
                  style={{
                    backgroundColor: 'rgba(255, 255, 255, 0.8)',
                    padding: '0.2rem 0.5rem',
                    borderRadius: '4px',
                    fontSize: '0.6rem',
                    transform: `scale(${scaleFactor})`,
                    transition: 'transform 0.6s cubic-bezier(0.2, 0.8, 0.2, 1)',
                  }}
                  onPointerOver={handlePointerOver}
                  onPointerOut={handlePointerOut}
                >
                  {text}
                </div>
              )}
            </Html>
          )}
        </mesh>
      </mesh>
    </>
  );
};
