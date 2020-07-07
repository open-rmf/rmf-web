import React from 'react';
import { Conflict, rawKnotsToKnots, Trajectory, RawKnot } from '../../robot-trajectory-manager';
import { bezierControlPoints, knotsToSegmentCoefficientsArray } from '../../util/cublic-spline';
import { TrajectoryPath } from './trajectory-animations';
import { useTheme } from '@material-ui/core';
import ColorManager from './colors';

export interface RobotTrajectoryProps
  extends React.RefAttributes<SVGPathElement>,
    React.SVGAttributes<SVGPathElement> {
  trajectory: Trajectory;
  conflicts: Conflict[];
  footprint: number;
  colorManager: Readonly<ColorManager>;
}

export const RobotTrajectory = React.forwardRef(function(
  props: RobotTrajectoryProps,
  ref: React.Ref<SVGPathElement>,
): React.ReactElement {
  const { trajectory, conflicts, footprint, colorManager, ...otherProps } = props;
  const theme = useTheme();

  const pathColor = React.useMemo(
    () =>
      conflicts.includes(trajectory.id)
        ? theme.palette.error.main
        : colorManager.robotColorFromCache(trajectory.robot_name),
    [trajectory, conflicts, theme, colorManager],
  );

  const pathD = React.useMemo(() => {
    return trajectoryPath(trajectory.segments).d;
  }, [trajectory]);

  return (
    <>
      {pathColor && (
        <path
          data-component="RobotTrajectory"
          ref={ref}
          d={pathD}
          stroke={pathColor}
          opacity={0.8}
          strokeWidth={footprint * 0.8}
          strokeLinecap="round"
          fill={'none'}
          pathLength={1}
          strokeDasharray={2}
          strokeDashoffset={0}
          {...otherProps}
        />
      )}
    </>
  );
});

export default RobotTrajectory;

export function trajectoryPath(trajectorySegments: RawKnot[]): TrajectoryPath {
  const knots = rawKnotsToKnots(trajectorySegments);
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
    d,
    segOffsets,
  };
}
