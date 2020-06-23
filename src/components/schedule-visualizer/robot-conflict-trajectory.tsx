import { useTheme } from '@material-ui/core';
import React from 'react';
import { RawKnot } from '../../robot-trajectory-manager';
import { trajectoryPath } from './robot-trajectory';

export interface RobotConflictTrajectoryProps {
  conflictsSegments?: RawKnot[];
  footprint: number;
}

export const RobotConflictTrajectory = React.forwardRef(function(
  props: RobotConflictTrajectoryProps,
  ref: React.Ref<SVGPathElement>,
): React.ReactElement {
  const { footprint, conflictsSegments, ...otherProps } = props;
  const theme = useTheme();
  const pathConflict = React.useMemo(() => {
    if (conflictsSegments && conflictsSegments.length !== 0) {
      return trajectoryPath(conflictsSegments).d;
    }
  }, [conflictsSegments]);

  return (
    <path
      ref={ref}
      d={pathConflict}
      stroke={theme.palette.error.main}
      opacity={0.8}
      strokeWidth={footprint * 0.5}
      strokeLinecap="round"
      fill={'none'}
      pathLength={1}
      strokeDasharray={2}
      strokeDashoffset={0}
      {...otherProps}
    />
  );
});

export default RobotConflictTrajectory;
