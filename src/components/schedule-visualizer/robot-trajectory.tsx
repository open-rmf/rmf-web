import { useTheme } from '@material-ui/core';
import Debug from 'debug';
import React from 'react';
import { Conflict, RawKnot, rawKnotsToKnots, Trajectory } from '../../robot-trajectory-manager';
import { bezierControlPoints, knotsToSegmentCoefficientsArray } from '../../util/cublic-spline';
import { TrajectoryPath } from './trajectory-animations';
import ColorManager from './colors';
import { TrajectoryDiameter, TrajectoryColor, TrajectoryAnimation } from '../../settings';
import { SettingsContext } from '../app-contexts';
import RobotTrajectoryConflict from './robot-trajectory-conflict';

const debug = Debug('ScheduleVisualizer:RobotTrajectory');

export interface RobotTrajectoryProps
  extends React.RefAttributes<SVGPathElement>,
    React.SVGAttributes<SVGPathElement> {
  trajectory: Trajectory;
  conflicts: Conflict[];
  footprint: number;
  colorManager?: Readonly<ColorManager>;
}

export const RobotTrajectory = React.memo(
  React.forwardRef(function(
    props: RobotTrajectoryProps,
    ref: React.Ref<SVGPathElement>,
  ): React.ReactElement {
    debug('render');

    const { trajectory, conflicts, footprint, colorManager, ...otherProps } = props;
    const theme = useTheme();
    const isConflict = conflicts.flat().includes(trajectory.id);
    const settings = React.useContext(SettingsContext);
    const trajDiameter = settings.trajectoryDiameter;
    const isOutline = settings.trajectoryAnimation === TrajectoryAnimation.Outline;

    function determineTrajDiameter(trajDiameter: TrajectoryDiameter): number {
      switch (trajDiameter) {
        case TrajectoryDiameter.FixSize:
          return 0.4;
        case TrajectoryDiameter.RobotSize:
          return footprint;
      }
    }

    const trajectoryDiameter = determineTrajDiameter(trajDiameter);

    const color = React.useMemo(() => {
      const getRobotColor = () => {
        const robotColor = colorManager?.robotColorFromCache(
          trajectory.fleet_name,
          trajectory.robot_name,
        );
        return !!robotColor ? robotColor : theme.palette.success.main;
      };
      const getPathColor = () => {
        const pathColor = colorManager?.pathColorFromCache(
          trajectory.fleet_name,
          trajectory.robot_name,
        );
        return !!pathColor ? pathColor : theme.palette.success.main;
      };
      const robotColorHolder = getRobotColor();
      const pathColorHolder = getPathColor();
      switch (settings.trajectoryColor) {
        case TrajectoryColor.Theme:
          return theme.palette.success.main;
        case TrajectoryColor.RobotColor:
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
        {color && (
          <path
            id="robotTrajectoryPath"
            data-component="RobotTrajectory"
            ref={ref}
            d={pathD}
            stroke={color}
            opacity={0.8}
            strokeWidth={isConflict && !isOutline ? trajectoryDiameter - 0.1 : trajectoryDiameter}
            strokeLinecap="round"
            fill={'none'}
            pathLength={1}
            strokeDasharray={2}
            strokeDashoffset={0}
            {...otherProps}
          />
        )}
        {isConflict && !isOutline ? (
          <RobotTrajectoryConflict
            pathD={pathD}
            trajectory={trajectory}
            trajectoryDiameter={trajectoryDiameter}
          />
        ) : null}
      </>
    );
  }),
);

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
