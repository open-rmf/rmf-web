import React from 'react';
import { useLoader } from '@react-three/fiber';
import { TextureLoader, Vector3 } from 'three';
import { Level } from 'api-client';
import { findSceneBoundingBoxFromThreeFiber } from 'react-components';

export interface ImageThreeProps {
  level: Level;
  imageUrl: string;
}

export const ImageThree = ({ level, imageUrl }: ImageThreeProps) => {
  const texture = useLoader(TextureLoader, imageUrl);
  const imageWidth = texture.image.width;
  const imageHeight = texture.image.height;
  const scaleX = imageWidth * level.images[0].scale + level.images[0].x_offset;
  const scaleY = imageHeight * level.images[0].scale + level.images[0].y_offset;

  const [sceneBoundingBox, setSceneBoundingBox] = React.useState<THREE.Box3 | undefined>(undefined);

  const [center, setCenter] = React.useState<Vector3>(
    new Vector3(level.images[0].x_offset, level.images[0].y_offset, level.images[0].yaw),
  );

  React.useMemo(() => {
    setSceneBoundingBox(findSceneBoundingBoxFromThreeFiber(level));
  }, [level]);

  React.useEffect(() => {
    if (!sceneBoundingBox) {
      return;
    }
    const centerBounding = sceneBoundingBox.getCenter(new Vector3());
    const test = new Vector3(centerBounding.x + 1.7, centerBounding.y, 0);

    setCenter(test);
  }, [sceneBoundingBox]);

  return sceneBoundingBox ? (
    <mesh scale={[scaleX, scaleY, 0]} position={center}>
      <planeGeometry />
      <meshBasicMaterial map={texture} />
    </mesh>
  ) : (
    <></>
  );
};
