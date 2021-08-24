import Debug from 'debug';
import React from 'react';
import { Trajectory, trajectoryPath } from './trajectory';
import { FollowAnimationPath } from './trajectory-paths';

const debug = Debug('Robots:TrajectoryMarker');

export type Conflict = number[];

export interface TrajectoryMarkerProps {
  trajectory: Trajectory;
  conflict: boolean;
  color: string;
  /**
   * default: false
   */
  animationLoop?: boolean;
  /**
   * default: 1
   */
  animationScale?: number;
}

export const TrajectoryMarker = React.forwardRef(
  (props: TrajectoryMarkerProps, ref: React.Ref<SVGGElement>) => {
    const {
      trajectory,
      conflict,
      color,
      animationLoop = false,
      animationScale = 1,
      ...otherProps
    } = props;
    debug(`render ${trajectory.id}`);
    const footprint = trajectory.dimensions;

    const pathD = React.useMemo(() => {
      return trajectoryPath(trajectory.segments).d;
    }, [trajectory]);

    return (
      <g ref={ref} {...otherProps}>
        <FollowAnimationPath
          trajectory={trajectory}
          d={pathD}
          color={color}
          footprint={footprint}
          conflict={conflict}
          animationLoop={animationLoop}
          animationScale={animationScale}
        />
      </g>
    );
  },
);

export default TrajectoryMarker;
