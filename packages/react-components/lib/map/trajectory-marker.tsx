import Debug from 'debug';
import React from 'react';
import { Trajectory, trajectoryPath } from './trajectory';
import { FollowAnimationPath } from './trajectory-paths';

const debug = Debug('Map:TrajectoryMarker');

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

export const TrajectoryMarker = React.forwardRef(
  (
    {
      trajectory,
      color,
      conflict = false,
      loopAnimation = false,
      animationScale = 1,
      ...otherProps
    }: TrajectoryMarkerProps,
    ref: React.Ref<SVGGElement>,
  ) => {
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
          loopAnimation={loopAnimation}
          animationScale={animationScale}
        />
      </g>
    );
  },
);

export default TrajectoryMarker;
