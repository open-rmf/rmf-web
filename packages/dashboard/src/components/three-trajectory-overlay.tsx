import { Line } from '@react-three/drei';
import * as THREE from 'three';

interface TrajectoryComponentProps {
  points: THREE.Vector3[];
  color: string;
}

export const TrajectoryComponent: React.FC<TrajectoryComponentProps> = ({ points, color }) => {
  return <Line points={points} color={color} linewidth={5} />;
};
