import React from 'react';
import { Trajectory } from './trajectory';

export interface TrajectoryMarkerProps extends React.PropsWithRef<{}> {
  trajectory: Trajectory;
  color: string;
  conflict?: boolean;
  loopAnimation?: boolean;
  /**
   * default: 1
   */
  animationScale?: number;
}
