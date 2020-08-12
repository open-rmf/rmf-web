import React, { useState } from 'react';
import { Conflict, rawKnotsToKnots, Trajectory, RawKnot } from '../../robot-trajectory-manager';
import { bezierControlPoints, knotsToSegmentCoefficientsArray } from '../../util/cublic-spline';
import { TrajectoryPath } from './trajectory-animations';
import { useTheme } from '@material-ui/core';
import ColorManager from './colors';

// @ts-ignore
import stringify from 'virtual-dom-stringify';
// @ts-ignore
import patterns from 'svg-patterns';

export interface RobotTrajectoryProps
  extends React.RefAttributes<SVGPathElement>,
    React.SVGAttributes<SVGPathElement> {
  trajectory: Trajectory;
  conflicts: Conflict[];
  footprint: number;
  colorManager?: Readonly<ColorManager>;
}

export const RobotTrajectory = React.forwardRef(function(
  props: RobotTrajectoryProps,
  ref: React.Ref<SVGPathElement>,
): React.ReactElement {
  const { trajectory, conflicts, footprint, colorManager, ...otherProps } = props;
  const [pattern, setPattern] = useState(
    patterns.lines({
      stroke: 'white', // any SVG-compatible color
      background: '#343434', // any SVG-compatible color
      orientations: [45],
    }),
  );
  const theme = useTheme();

  function generatePattern(pathColor: string) {
    return patterns.lines({
      size: 0.8,
      strokeWidth: 0.07,
      stroke: 'black', // any SVG-compatible color
      background: pathColor, // any SVG-compatible color
      orientations: [45],
    });
  }

  const pathColor = React.useMemo(() => {
    const getRobotColor = () => {
      const robotColor = colorManager?.robotColorFromCache(trajectory.robot_name);
      return !!robotColor ? robotColor : theme.palette.success.main;
    };
    setPattern(
      generatePattern(
        conflicts.flat().includes(trajectory.id) ? theme.palette.error.main : getRobotColor(),
      ),
    );
    return conflicts.flat().includes(trajectory.id) ? theme.palette.error.main : getRobotColor();
  }, [trajectory, conflicts, theme, colorManager]);

  const pathD = React.useMemo(() => {
    return trajectoryPath(trajectory.segments).d;
  }, [trajectory]);

  return (
    <>
      <defs dangerouslySetInnerHTML={{ __html: stringify(pattern) }} />
      {pathColor && (
        <path
          data-component="RobotTrajectory"
          ref={ref}
          d={pathD}
          stroke={pattern.url()}
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
