import * as THREE from 'three';
import React, { Suspense } from 'react';
import { Level } from 'api-client';
import { graphToWalls, Wall } from './wall';
import { Door } from './door';
import { Canvas } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import './temporalStyle.css';

export interface LevelProps {
  levels: Level[];
}

export interface BuildingMapProps {
  level: Level;
}

const BuildingMap = ({ level }: BuildingMapProps) => {
  const { doors, elevation } = level;

  return (
    <>
      {level.wall_graph.edges.length > 0 ? (
        <Wall elevation={elevation} opacity={1} walls={level.wall_graph} />
      ) : null}

      {doors.length &&
        doors.map((door, i) => (
          <Door key={i} door={door} opacity={0.1} height={8} elevation={elevation} />
        ))}
    </>
  );
};

const findSceneBoundingBox = (level: Level): THREE.Box3 => {
  let minX = Infinity;
  let minY = Infinity;
  let minZ = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  let maxZ = -Infinity;

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

  return new THREE.Box3(new THREE.Vector3(minX, minY, minZ), new THREE.Vector3(maxX, maxY, maxZ));
};

export function MapConstruction({ levels }: LevelProps) {
  const ref = React.useRef(null);
  const [lerping, setLerping] = React.useState(false);
  const [to, setTo] = React.useState();
  const [target, setTarget] = React.useState();
  const [selected, setSelected] = React.useState(-1);
  const [currentLevel, setCurrentLevel] = React.useState<Level>(levels[0]);
  const [sceneBoundingBox, setSceneBoundingBox] = React.useState<THREE.Box3 | undefined>(undefined);
  const [distance, setDistance] = React.useState<number>(0);

  // React.useMemo(() => setSceneBoundingBox(findSceneBoundingBox(currentLevel)), [currentLevel]);

  // React.useEffect(() => {
  //   if (!sceneBoundingBox) {
  //     return
  //   }

  //   const size = sceneBoundingBox.getSize(new THREE.Vector3());
  //   setDistance(Math.max(size.x, size.y, size.z) * 0.9);

  // }, [sceneBoundingBox])

  const [overallSceneBoundingBox, setOverallSceneBoundingBox] = React.useState<
    THREE.Box3 | undefined
  >(undefined);

  React.useMemo(() => {
    const combinedBoundingBox = new THREE.Box3();
    levels.forEach((level) => {
      const levelBoundingBox = findSceneBoundingBox(level);
      combinedBoundingBox.expandByPoint(levelBoundingBox.min);
      combinedBoundingBox.expandByPoint(levelBoundingBox.max);
    });
    setOverallSceneBoundingBox(combinedBoundingBox);
  }, [levels]);

  React.useEffect(() => {
    if (!overallSceneBoundingBox) {
      return;
    }

    const size = overallSceneBoundingBox.getSize(new THREE.Vector3());
    setDistance(Math.max(size.x, size.y, size.z) * 0.9);
  }, [overallSceneBoundingBox]);

  return (
    <Suspense fallback={null}>
      <Canvas
        onPointerDown={() => setLerping(false)}
        onWheel={() => setLerping(false)}
        onCreated={({ camera }) => {
          if (!overallSceneBoundingBox) {
            return;
          }
          const center = overallSceneBoundingBox.getCenter(new THREE.Vector3());
          camera.position.set(center.x, center.y, center.z + distance);
          camera.updateProjectionMatrix();
        }}
      >
        {/* <OrbitControls
          ref={ref}
          enableZoom
          enablePan
          enableRotate
          target={sceneBoundingBox?.getCenter(new THREE.Vector3())}
          maxDistance={distance}
        /> */}
        <OrbitControls
          target={overallSceneBoundingBox?.getCenter(new THREE.Vector3())}
          ref={ref}
          enableZoom
          enablePan
          enableDamping
          dampingFactor={0.1}
        />
        <BuildingMap level={currentLevel} />
        <ambientLight />
      </Canvas>
      <div id="annotationsPanel">
        <ul>
          {levels.map((level, i) => {
            const { name } = level;
            return (
              <li key={i}>
                <button
                  className="annotationButton"
                  name={name}
                  onClick={(event: React.MouseEvent<HTMLButtonElement>) => {
                    setCurrentLevel(
                      levels.find((l) => l.name === event.currentTarget?.name) || currentLevel,
                    );
                  }}
                >
                  {name}
                </button>
              </li>
            );
          })}
        </ul>
      </div>
    </Suspense>
  );
}
