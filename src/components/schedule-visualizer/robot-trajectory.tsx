import { useTheme } from '@material-ui/core';
import React, { useState } from 'react';
import { Conflict, rawKnotsToKnots, Trajectory, RawKnot } from '../../robot-trajectory-manager';
import { bezierControlPoints, knotsToSegmentCoefficientsArray } from '../../util/cublic-spline';
import ColorManager from './colors';

export interface RobotTrajectoryProps
  extends React.RefAttributes<SVGPathElement>,
    React.SVGAttributes<SVGPathElement> {
  trajectory: Trajectory;
  conflicts: Conflict[];
  conflictsSegments?: RawKnot[];
  footprint: number;
}

export const RobotTrajectory = React.forwardRef(function(
  props: RobotTrajectoryProps,
  ref: React.Ref<SVGPathElement>,
): React.ReactElement {
  const { trajectory, conflicts, footprint, conflictsSegments, ...otherProps } = props;
  const theme = useTheme();

  const [pathColor] = useState(ColorManager.getPathColor(trajectory.id));
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

  const pathConflict = React.useMemo(() => {
    if (conflictsSegments && conflictsSegments.length !== 0) {
      const knots = rawKnotsToKnots(conflictsSegments);
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
    }
  }, [conflictsSegments]);

  return (
    <>
      {pathConflict && pathColor && (
        <path
          ref={ref}
          d={pathConflict}
          stroke={theme.palette.error.main}
          opacity={0.8}
          strokeWidth={footprint * 0.6}
          strokeLinecap="round"
          fill="none"
          pathLength={1}
          strokeDasharray={2}
          strokeDashoffset={0}
          {...otherProps}
        />
      )}
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

// export function trajectoryPath(trajectory: Trajectory): TrajectoryPath {
//   const knots = rawKnotsToKnots(trajectory.segments);
//   const coeff = knotsToSegmentCoefficientsArray(knots);
//   const bezierSplines = coeff.map(bezierControlPoints);

//   const totalDuration = knots[knots.length - 1].time - knots[0].time;
//   const segOffsets = knots.map(k => (k.time - knots[0].time) / totalDuration);

//   let d = `M ${bezierSplines[0][0][0]} ${-bezierSplines[0][0][1]} C `;
//   bezierSplines.map(
//     bzCurves =>
//       (d +=
//         `${bzCurves[1][0]} ${-bzCurves[1][1]} ` +
//         `${bzCurves[2][0]} ${-bzCurves[2][1]} ` +
//         `${bzCurves[3][0]} ${-bzCurves[3][1]} `),
//   );

//   return {
//     traj: trajectory,
//     d,
//     segOffsets,
//   };
// }
