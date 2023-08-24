import React from 'react';
import { DoorState, Lift, LiftState } from 'api-client';
import { Cube } from './cube';
import { Door as DoorModel } from 'rmf-models';
import { RmfAppContext } from './rmf-app';
import { DoorMode } from 'rmf-models';
import { Text, Line } from '@react-three/drei';
import { getLiftModeText } from 'react-components';
import { BufferGeometry, BufferAttribute, Vector3, Euler } from 'three';
import { LiftState as RmfLiftState } from 'rmf-models';

interface DoorProps {
  door: DoorModel;
  opacity: number;
  height: number;
  elevation: number;
  lift?: Lift;
}

interface SquareProps {
  x: number;
  y: number;
  yaw: number;
  width: number;
  depth: number;
  liftState: LiftState;
}

interface LiftShapeMakerProps {
  motionState: number;
}

function distance(v1_x: number, v1_y: number, v2_x: number, v2_y: number) {
  return Math.hypot(v2_x - v1_x, v2_y - v1_y);
}

function midPoint(v1_x: number, v1_y: number, v2_x: number, v2_y: number) {
  return [(v2_x + v1_x) / 2, (v2_y + v1_y) / 2];
}

const LiftShapeMaker = ({ motionState }: LiftShapeMakerProps) => {
  const vertices = new Float32Array([0, 1, 0, -0.5, -0.5, 0, 0.5, -0.5, 0]);

  const generateTriangleShape = (rotation: Euler) => {
    const geometry = new BufferGeometry();
    geometry.setAttribute('position', new BufferAttribute(vertices, 3));
    return (
      <mesh geometry={geometry} rotation={rotation}>
        <meshBasicMaterial color="red" />
      </mesh>
    );
  };

  const generateLineShape = () => {
    const points = [
      [-0.5, 0.5, 0],
      [0.5, 0.5, 0],
      [0.5, -0.5, 0],
      [-0.5, -0.5, 0],
      [-0.5, 0.5, 0],
    ].map((point) => new Vector3(...point));

    return <Line points={points} color="black" linewidth={1} />;
  };

  const generateTextShape = () => {
    return (
      <Text color="black" fontSize={0.6}>
        {' '}
        {'?'}
      </Text>
    );
  };

  let shapeComponent;

  switch (motionState) {
    case RmfLiftState.MOTION_UP:
      shapeComponent = generateTriangleShape(new Euler(0, 0, 0));
      break;
    case RmfLiftState.MOTION_DOWN:
      shapeComponent = generateTriangleShape(new Euler(0, 0, Math.PI));
      break;
    case RmfLiftState.MOTION_STOPPED:
      shapeComponent = generateLineShape();
      break;
    default:
      shapeComponent = generateTextShape();
      break;
  }

  return shapeComponent;
};

const ElevatorMaker = ({ x, y, yaw, width, depth, liftState }: SquareProps) => {
  return (
    <group position={[x, y, yaw]}>
      <Text position={[0, 0.8, 0.5]} color="black" fontSize={0.6}>
        {getLiftModeText(liftState)}
      </Text>
      <LiftShapeMaker motionState={liftState.motion_state} />
      <mesh position={[0, 0, 0]} rotation={[0, 0, yaw]}>
        <boxGeometry args={[width, depth, 0.1]} />
        <meshStandardMaterial color={'green'} opacity={0.6} transparent />
      </mesh>
    </group>
  );
};

interface SingleDoorProps extends DoorProps {
  door: DoorModel;
  opacity: number;
  meshRef: React.Ref<THREE.Mesh>;
  doorState: number | undefined;
  color: string;
  liftSate?: LiftState;
}

function SingleSwingDoor({
  meshRef,
  opacity,
  door,
  height,
  doorState,
  elevation,
  color,
}: SingleDoorProps) {
  const { v1_x, v1_y, v2_x, v2_y } = door;
  const thickness = 0.5;
  const v = new Vector3(v1_x - v2_x, 0, v1_y - v2_y);
  v.normalize();
  const angle = Math.atan2(v1_y - v2_y, v1_x - v2_x) - Math.PI / 2;
  const rot = new Euler(0, 0, angle);

  const pos = midPoint(v1_x, v1_y, v2_x, v2_y).concat(height / 2 + 0);
  const dist = distance(v1_x, v1_y, v2_x, v2_y);
  return (
    <Cube
      meshRef={meshRef}
      key={door.name}
      position={pos}
      size={[thickness, dist, height]}
      rot={rot}
      color={color}
    />
  );
}

function toDoorMode(liftState: LiftState): DoorMode {
  return { value: liftState.door_state };
}

export const Door = React.memo(({ ...doorProps }: DoorProps): JSX.Element => {
  const ref = React.useRef<THREE.Mesh>(null!);
  const { door, lift } = doorProps;
  const rmf = React.useContext(RmfAppContext);
  const [doorState, setDoorState] = React.useState<DoorState | null>(null);
  const [liftState, setLiftState] = React.useState<LiftState | undefined>(undefined);
  const [color, setColor] = React.useState<string>('red');

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    if (!lift) {
      return;
    }

    const sub = rmf.getLiftStateObs(lift.name).subscribe(setLiftState);
    return () => sub.unsubscribe();
  }, [rmf, lift]);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = rmf.getDoorStateObs(door.name).subscribe(setDoorState);
    return () => sub.unsubscribe();
  }, [rmf, door.name]);

  React.useEffect(() => {
    let doorStateValue = doorState?.current_mode.value;
    if (liftState) {
      doorStateValue = toDoorMode(liftState).value;
    }
    switch (doorStateValue) {
      case DoorMode.MODE_CLOSED:
        setColor('red');
        return;
      case DoorMode.MODE_MOVING:
        setColor('orange');
        return;
      case DoorMode.MODE_OPEN:
      default:
        setColor('green');
        return;
    }
  }, [doorState?.current_mode.value, liftState]);

  return (
    <>
      <SingleSwingDoor
        {...doorProps}
        key={door.name}
        meshRef={ref}
        doorState={doorState?.current_mode.value}
        color={color}
      />
      {lift && liftState && (
        <ElevatorMaker
          x={lift.ref_x}
          y={lift.ref_y}
          yaw={lift.ref_yaw}
          width={lift.width}
          depth={lift.depth}
          liftState={liftState}
        />
      )}
    </>
  );
});
