import { Line, Text } from '@react-three/drei';
import { ThreeEvent } from '@react-three/fiber';
import { LiftState } from 'api-client';
import { LiftState as RmfLiftState } from 'rmf-models/ros/rmf_lift_msgs/msg';
import { BufferAttribute, BufferGeometry, Euler, Vector3 } from 'three';

// Gets the text to insert to the lift, the text depend on the current mode,
// motion state and the current and destination floor of the lift.
export const getLiftModeText = (liftState: LiftState): string => {
  if (!liftState.current_mode) {
    return 'UNKNOWN';
  }
  switch (liftState.current_mode) {
    case RmfLiftState.MODE_FIRE:
      return 'FIRE!';
    case RmfLiftState.MODE_EMERGENCY:
      return 'EMERGENCY!';
    case RmfLiftState.MODE_OFFLINE:
      return 'OFFLINE';
    default:
      return 'NORMAL';
  }
};

interface LiftMakerProps {
  x: number;
  y: number;
  yaw: number;
  width: number;
  depth: number;
  liftState: LiftState;
  fontPath?: string;
  onLiftClick?: (ev: ThreeEvent<MouseEvent>) => void;
}

interface LiftShapeMakerProps {
  motionState: number;
  fontPath?: string;
}

const LiftShapeMaker = ({ motionState, fontPath }: LiftShapeMakerProps) => {
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

  const generateTextShape = (fontPath?: string) => {
    return fontPath && fontPath.length > 0 ? (
      <Text color="black" font={fontPath} fontSize={0.6}>
        {' '}
        {'?'}
      </Text>
    ) : null;
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
      shapeComponent = generateTextShape(fontPath);
      break;
  }

  return shapeComponent;
};

export const LiftThreeMaker = ({
  x,
  y,
  yaw,
  width,
  depth,
  liftState,
  fontPath,
  onLiftClick,
}: LiftMakerProps): JSX.Element => {
  return (
    <group position={[x, y, yaw]}>
      {fontPath && fontPath.length > 0 ? (
        <>
          <Text color="black" font={fontPath} fontSize={0.6}>
            {liftState.current_floor}
          </Text>
          <Text position={[0, 0.8, 0.5]} color="black" font={fontPath} fontSize={0.6}>
            {getLiftModeText(liftState)}
          </Text>
        </>
      ) : null}
      <LiftShapeMaker motionState={liftState.motion_state} fontPath={fontPath} />
      <mesh
        position={[0, 0, 0]}
        rotation={[0, 0, yaw]}
        onClick={(ev: ThreeEvent<MouseEvent>) => onLiftClick && onLiftClick(ev)}
      >
        <boxGeometry args={[width, depth, 0.1]} />
        <meshStandardMaterial color={'green'} opacity={0.6} transparent />
      </mesh>
    </group>
  );
};
