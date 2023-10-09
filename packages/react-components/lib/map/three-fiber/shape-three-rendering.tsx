import { Circle, Html } from '@react-three/drei';
import React from 'react';

interface ShapeThreeRenderingProps {
  position: [number, number, number];
  color: string;
  text?: string;
  circleShape: boolean;
}

// eslint-disable-next-line @typescript-eslint/no-explicit-any
export const debounce = <T extends (...args: any[]) => void>(
  callback: T,
  delay: number,
): ((...args: Parameters<T>) => void) => {
  let timeoutId: NodeJS.Timeout | null = null;
  return (...args: Parameters<T>) => {
    if (timeoutId) {
      clearTimeout(timeoutId);
    }
    timeoutId = setTimeout(() => {
      callback(...args);
    }, delay);
  };
};

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

  const debouncedHandlePointerOver = debounce(() => {
    setIsHovered(true);
  }, 300);

  const debouncedHandlePointerOut = debounce(() => {
    setIsHovered(false);
  }, 300);

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
            position={[0, 0, positionZ]}
            scale={[0.7, 0.7, 0.7]}
            onPointerOver={debouncedHandlePointerOver}
            onPointerOut={debouncedHandlePointerOut}
          >
            {isHovered && (
              <Html zIndexRange={[1]}>
                <div
                  style={{
                    backgroundColor: 'rgba(255, 255, 255, 0.8)',
                    padding: '0.2rem 0.5rem',
                    borderRadius: '4px',
                    fontSize: '0.6rem',
                    transform: `scale(${scaleFactor})`,
                    transition: 'transform 0.3s',
                  }}
                >
                  {text}
                </div>
              </Html>
            )}
            <boxGeometry args={[1.3, 1.3, 1.3]} />
            <meshStandardMaterial color={color} opacity={1} />
          </mesh>
        </group>
      )}
    </>
  );
};
