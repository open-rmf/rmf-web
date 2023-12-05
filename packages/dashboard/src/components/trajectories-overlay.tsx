import { Trajectory } from 'react-components';

export interface TrajectoryData {
  trajectory: Trajectory;
  color: string;
  conflict?: boolean;
  loopAnimation?: boolean;
  /**
   * default: 1
   */
  animationScale?: number;
}
