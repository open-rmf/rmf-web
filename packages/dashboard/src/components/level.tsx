import * as THREE from 'three';
import React, { Suspense } from 'react';
import { MeshProps } from '@react-three/fiber';
import { Graph, GraphNode, Level } from 'api-client';
import { Wall } from './wall';
import { Door } from './door';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import {
  PerspectiveCamera,
  PositionalAudio,
  OrbitControls,
  Environment,
  Stats,
  Stage,
} from '@react-three/drei';

export interface LevelProps {
  levels: Level[];
}

export interface BuildingMapProps {
  level: Level;
  show: boolean;
}

interface BoundingBoxProps {
  walls: Graph;
  opacity: number;
  elevation: number;
}

interface WallSegment {
  position: number[];
  width: number;
  height: number;
  depth: number;
  rot: THREE.Euler;
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

const distance = (v1: GraphNode, v2: GraphNode) => Math.hypot(v2.x - v1.x, v2.y - v1.y);
const midPoint = (v1: GraphNode, v2: GraphNode) => [(v2.x + v1.x) / 2, (v2.y + v1.y) / 2];

const graphToWalls = (graph: Graph, elevation: number) => {
  const walls = [] as WallSegment[];
  const { edges, vertices } = graph;
  let minX = Infinity;
  let minY = Infinity;
  let minZ = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  let maxZ = -Infinity;

  edges.map((edge) => {
    const v1 = vertices[edge.v1_idx];
    const v2 = vertices[edge.v2_idx];

    const depth = distance(v1, v2);
    const midpoint = midPoint(v1, v2);
    const width = 0.3;
    const height = 8;
    const position = midpoint.concat(height / 2 + elevation);

    const angle = Math.atan2(v1.y - v2.y, v1.x - v2.x) - Math.PI / 2;
    const rot = new THREE.Euler(0, 0, angle);

    walls.push({
      position,
      width,
      height,
      depth,
      rot,
    });
    minX = Math.min(minX, v1.x, v2.x);
    minY = Math.min(minY, v1.y, v2.y);
    minZ = Math.min(minZ, position[2] - height / 2);
    maxX = Math.max(maxX, v1.x, v2.x);
    maxY = Math.max(maxY, v1.y, v2.y);
    maxZ = Math.max(maxZ, position[2] + height / 2);
  });

  const boundingBox = new THREE.Box3(
    new THREE.Vector3(minX, minY, minZ),
    new THREE.Vector3(maxX, maxY, maxZ),
  );

  return { walls, boundingBox };
};

const BoundingBox = ({ walls, elevation, opacity }: BoundingBoxProps) => {
  const walls_ = graphToWalls(walls, elevation);
  const { camera } = useThree();
  const boundingBoxRef = React.useRef<THREE.Box3>();
  const controlsRef = React.useRef<any>();

  React.useEffect(() => {
    if (walls_.boundingBox !== boundingBoxRef.current) {
      boundingBoxRef.current = walls_.boundingBox;
    }
  }, [walls_]);

  useFrame(() => {
    if (boundingBoxRef.current) {
      const center = boundingBoxRef.current.getCenter(new THREE.Vector3());
      const size = boundingBoxRef.current.getSize(new THREE.Vector3());
      const maxDim = Math.max(size.x, size.y, size.z);

      // Calculate the zoom level based on the bounding box size
      const fov =
        camera instanceof THREE.PerspectiveCamera
          ? camera.fov * (Math.PI / 180)
          : 45 * (Math.PI / 180);
      const distance = Math.abs(maxDim / (2 * Math.tan(fov / 2)));

      // Calculate the target position for the camera
      // const target = center.clone().add(camera.position.clone().sub(center).normalize().multiplyScalar(distance));
      // camera.position.copy(target);
      // camera.lookAt(center);
    }
  });

  // useFrame(() => {
  //   if (boundingBoxRef.current) {
  //     const center = boundingBoxRef.current.getCenter(new THREE.Vector3());
  //     camera.lookAt(center);
  //   }
  // });
  // useFrame(({ camera }) => {
  //   if (controlsRef.current) {
  //     controlsRef.current.target = camera.position;
  //   }
  // });

  return null;

  // return (
  //   <>
  //     <OrbitControls ref={controlsRef} />
  //   </>
  // );
};

const BuildingMap = ({ level, show }: BuildingMapProps, props: MeshProps) => {
  const { doors, elevation } = level;

  return (
    <>
      {level.wall_graph.edges.length > 0 ? (
        <>
          <BoundingBox elevation={elevation} opacity={Number(show)} walls={level.wall_graph} />
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

export function MapConstruction({ levels }: LevelProps) {
  const ref = React.useRef(null);
  const [lerping, setLerping] = React.useState(false);
  const [to, setTo] = React.useState();
  const [target, setTarget] = React.useState();
  const [selected, setSelected] = React.useState(-1);

  return (
    <Suspense fallback={null}>
      <Canvas
        onPointerDown={() => setLerping(false)}
        onWheel={() => setLerping(false)}
        camera={{ position: [12, 1, 22] }}
        onCreated={({ camera }) => {
          camera.zoom = 1;
          camera.updateProjectionMatrix();
        }}
      >
        <OrbitControls ref={ref} enableZoom enablePan enableRotate />
        {levels.map((level, i) => (
          <BuildingMap key={i} level={level} show={true} />
        ))}
        <Animate controls={ref} lerping={lerping} to={to} target={target} />
        <ambientLight />
      </Canvas>
    </Suspense>
  );
}
