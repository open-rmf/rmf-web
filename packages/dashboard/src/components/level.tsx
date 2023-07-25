import * as THREE from 'three';
import React, { Suspense } from 'react';
import { MeshProps } from '@react-three/fiber';
import { Level } from 'api-client';
import { graphToWalls, Wall } from './wall';
import { Door } from './door';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';

export interface LevelProps {
  levels: Level[];
}

export interface BuildingMapProps {
  level: Level;
  show: boolean;
}

function Animate({ controls, lerping, to, target }: any) {
  useFrame(({ camera }, delta) => {
    if (lerping) {
      const boundingBox = new THREE.Box3().setFromObject(target);
      const center = boundingBox.getCenter(new THREE.Vector3());

      camera.position.lerp(to, delta * 2);
      controls.current.target.lerp(center, delta * 2);
    }
  });

  return null;
}

const BuildingMap = ({ level, show }: BuildingMapProps, props: MeshProps) => {
  const { doors, elevation } = level;

  return (
    <>
      {level.wall_graph.edges.length > 0 ? (
        <>
          <Wall elevation={elevation} opacity={Number(show)} walls={level.wall_graph} />
        </>
      ) : null}

      {doors.length &&
        doors.map((door, i) => (
          <Door key={i} door={door} opacity={0.1} height={8} elevation={elevation} />
        ))}
    </>
  );
};

const findSceneBoundingBox = (levels: Level[]) => {
  let minX = Infinity;
  let minY = Infinity;
  let minZ = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  let maxZ = -Infinity;

  levels.forEach((level) => {
    const walls = graphToWalls(level.wall_graph, level.elevation);
    walls.forEach((wall) => {
      const [x, y, z] = wall.position;
      const width = wall.width;
      const height = wall.height;

      minX = Math.min(minX, x - width / 2);
      minY = Math.min(minY, y - width / 2);
      minZ = Math.min(minZ, z - height / 2);
      maxX = Math.max(maxX, x + width / 2);
      maxY = Math.max(maxY, y + width / 2);
      maxZ = Math.max(maxZ, z + height / 2);
    });
  });

  return new THREE.Box3(new THREE.Vector3(minX, minY, minZ), new THREE.Vector3(maxX, maxY, maxZ));
};

export function MapConstruction({ levels }: LevelProps) {
  const ref = React.useRef(null);
  const [lerping, setLerping] = React.useState(false);
  const [to, setTo] = React.useState();
  const [target, setTarget] = React.useState();
  const [selected, setSelected] = React.useState(-1);

  const sceneBoundingBox = React.useMemo(() => findSceneBoundingBox(levels), [levels]);
  const size = sceneBoundingBox.getSize(new THREE.Vector3());
  const distance = Math.max(size.x, size.y, size.z) * 0.9;

  return (
    <Suspense fallback={null}>
      <Canvas
        onPointerDown={() => setLerping(false)}
        onWheel={() => setLerping(false)}
        onCreated={({ camera }) => {
          const center = sceneBoundingBox.getCenter(new THREE.Vector3());
          camera.position.set(center.x, center.y, center.z + distance);
          camera.updateProjectionMatrix();
        }}
      >
        <OrbitControls
          ref={ref}
          enableZoom
          enablePan
          enableRotate
          target={sceneBoundingBox.getCenter(new THREE.Vector3())}
          maxDistance={distance}
        />
        {levels.map((level, i) => (
          <BuildingMap key={i} level={level} show={true} />
        ))}
        <ambientLight />
      </Canvas>
    </Suspense>
  );
}
