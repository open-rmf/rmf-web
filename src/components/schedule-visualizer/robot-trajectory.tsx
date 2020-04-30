import { useTheme } from '@material-ui/core';
import React from 'react';
import { Conflict, rawKnotsToKnots, Trajectory } from '../../robot-trajectory-manager';
import { bezierControlPoints, knotsToSegmentCoefficientsArray } from '../../util/cublic-spline';
import { TrajectoryPath } from './trajectory-animations';

export interface RobotTrajectoryProps
  extends React.RefAttributes<SVGPathElement>,
    React.SVGAttributes<SVGPathElement> {
  trajectory: Trajectory;
  conflicts: Conflict[];
  footprint: number;
}

export const RobotTrajectory = React.forwardRef(function(
  props: RobotTrajectoryProps,
  ref: React.Ref<SVGPathElement>,
): React.ReactElement {
  const { trajectory, conflicts, footprint, ...otherProps } = props;
  const theme = useTheme();

  const pathD = React.useMemo(() => {
    const knots = rawKnotsToKnots(trajectory.segments);
    const coeff = knotsToSegmentCoefficientsArray(knots);
    const bezierSplines = coeff.map(bezierControlPoints);

    let d = `M ${bezierSplines[0][0][0]} ${-bezierSplines[0][0][1]} C `;
    bezierSplines.map(
      bzCurves =>
        (d +=
          `${bzCurves[1][0]} ${-bzCurves[1][1]} ` +
          `${bzCurves[2][0]} ${-bzCurves[2][1]} ` +
          `${bzCurves[3][0]} ${-bzCurves[3][1]} `),
    );

    return d;
  }, [trajectory]);

  const color = React.useMemo(
    () =>
      conflicts.includes(trajectory.id) ? theme.palette.error.main : theme.palette.success.main,
    [trajectory, conflicts, theme],
  );

  return (
    <path
      ref={ref}
      d={pathD}
      stroke={color}
      opacity={0.8}
      strokeWidth={footprint * 0.8}
      strokeLinecap="round"
      fill="none"
      pathLength={1}
      strokeDasharray={2}
      strokeDashoffset={0}
      {...otherProps}
    />
  );
});

export default RobotTrajectory;

export function trajectoryPath(trajectory: Trajectory): TrajectoryPath {
  const knots = rawKnotsToKnots(trajectory.segments);
  const coeff = knotsToSegmentCoefficientsArray(knots);
  const bezierSplines = coeff.map(bezierControlPoints);

  const totalDuration = knots[knots.length - 1].time - knots[0].time;
  const segOffsets = knots.map(k => (k.time - knots[0].time) / totalDuration);

  let d = `M ${bezierSplines[0][0][0]} ${-bezierSplines[0][0][1]} C `;
  bezierSplines.map(
    bzCurves =>
      (d +=
        `${bzCurves[1][0]} ${-bzCurves[1][1]} ` +
        `${bzCurves[2][0]} ${-bzCurves[2][1]} ` +
        `${bzCurves[3][0]} ${-bzCurves[3][1]} `),
  );

  return {
    traj: trajectory,
    d,
    segOffsets,
  };
}
