import React from 'react';
import { Conflict, rawKnotsToKnots, Trajectory, RawKnot } from '../../robot-trajectory-manager';
import { bezierControlPoints, knotsToSegmentCoefficientsArray } from '../../util/cublic-spline';
import { TrajectoryPath } from './trajectory-animations';
import { useTheme } from '@material-ui/core';
import ColorManager from './colors';
import { SettingsContext, TrajectoryDiameter, TrajectoryColor } from '../../settings';

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

  const isConflict = conflicts.flat().includes(trajectory.id);
  const theme = useTheme();
  const settings = React.useContext(SettingsContext);
  const trajDiameter = settings.trajectoryDiameter;

  function determineTrajDiameter(trajDiameter: TrajectoryDiameter): number {
    switch (trajDiameter) {
      case TrajectoryDiameter.Fix_size:
        return 0.4;
      case TrajectoryDiameter.Robot_size:
        return footprint;
    }
  }

  const trajectoryDiameter = determineTrajDiameter(trajDiameter);
  const pathColor = React.useMemo(() => {
    const getRobotColor = () => {
      const robotColor = colorManager?.robotColorFromCache(trajectory.robot_name);
      return !!robotColor ? robotColor : theme.palette.success.main;
    };
    const getPathColor = () => {
      const pathColor = colorManager?.pathColorFromCache(trajectory.robot_name);
      return !!pathColor ? pathColor : theme.palette.success.main;
    };
    const robotColorHolder = getRobotColor();
    const pathColorHolder = getPathColor();
    switch (settings.trajectoryColor) {
      case TrajectoryColor.Theme:
        return theme.palette.success.main;
      case TrajectoryColor.Robot_Color:
        return robotColorHolder;
      case TrajectoryColor.Shades:
        return pathColorHolder;
    }
  }, [trajectory, theme, colorManager, settings.trajectoryColor]);

  const pathD = React.useMemo(() => {
    return trajectoryPath(trajectory.segments).d;
  }, [trajectory]);

  return (
    <>
      <filter
        id={`shadow`}
        x="-20%"
        y="-20%"
        width="140%"
        height="140%"
        filterUnits="userSpaceOnUse"
      >
        <feColorMatrix
          type="matrix"
          values="1 0 0 0 0 
                                      0 1 0 0 0 
                                      0 0 1 0 0 
                                      0 0 0 1 0"
          result="boostedInput"
        />

        <feGaussianBlur stdDeviation={footprint * 0.4} />
        <feComposite operator="out" in2="boostedInput" />
      </filter>
      {pathColor && (
        <path
          id="myPath"
          data-component="RobotTrajectory"
          ref={ref}
          d={pathD}
          stroke={pathColor}
          opacity={0.8}
          strokeWidth={trajectoryDiameter}
          strokeLinecap="round"
          fill={'none'}
          pathLength={1}
          strokeDasharray={2}
          strokeDashoffset={0}
          {...otherProps}
        />
      )}
      {isConflict ? (
        <path
          id="errorPath"
          d={pathD}
          stroke={theme.palette.secondary.main}
          strokeWidth={trajectoryDiameter}
          strokeLinecap="round"
          fill={'none'}
          filter={`url(#shadow)`}
          strokeDasharray={2}
          strokeDashoffset={0}
          pathLength={1}
          opacity={1}
        />
      ) : null}
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
