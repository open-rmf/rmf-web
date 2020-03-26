import * as RomiCore from '@osrf/romi-js-core-interfaces';
import * as L from 'leaflet';
import React from 'react';
import { rawKnotsToKnots, Trajectory } from '../../robot-trajectory-manager';
import { bezierControlPoints, knotsToSegmentCoefficientsArray } from '../../util/cublic-spline';
import ColorManager from './colors';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';

export interface RobotTrajectoriesOverlayProps extends SVGOverlayProps {
  trajs: readonly Trajectory[];
  colorManager: ColorManager;
  animationDuration: number;
}

export default function RobotTrajectoriesOverlay(
  props: RobotTrajectoriesOverlayProps,
): React.ReactElement {
  const { trajs, colorManager, animationDuration, ...otherProps } = props;
  const [pendingColors, setPendingColors] = React.useState<RomiCore.RobotState[]>([]);
  const pathRefs = React.useRef<Record<string, SVGPathElement | null>>({});

  const bounds =
    props.bounds instanceof L.LatLngBounds ? props.bounds : new L.LatLngBounds(props.bounds);
  const width = bounds.getEast() - bounds.getWest();
  const height = bounds.getNorth() - bounds.getSouth();
  const viewBox = `0 0 ${width} ${height}`;

  const trajPaths = React.useMemo(() => {
    return trajs.map<TrajectoryPath>(traj => {
      const knots = rawKnotsToKnots(traj.segments);
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
        traj,
        d,
        segOffsets,
      };
    });
  }, [trajs]);

  React.useEffect(() => {
    if (!pendingColors.length) {
      return;
    }
    (async () => {
      await Promise.all(
        pendingColors.map(async robot => colorManager.robotColor(robot.name, robot.model)),
      );
      setPendingColors([]);
    })();
  });

  React.useEffect(() => {
    trajPaths.forEach(trajPath => {
      const ref = pathRefs.current[trajPath.traj.id];
      if (!ref) {
        return;
      }

      ref.animate(
        trajPath.segOffsets.map<Keyframe>(offset => ({
          offset,
          strokeDashoffset: 1 - offset,
        })),
        { duration: animationDuration, easing: 'linear', fill: 'forwards' },
      );
    });
  }, [animationDuration, trajPaths]);

  // FIXME: hardcode for now, as the source of the footprint is expected to change.
  const footprint = 0.5;

  return (
    <SVGOverlay {...otherProps}>
      <svg viewBox={viewBox}>
        {trajPaths.map(trajPath => (
          <g key={trajPath.traj.id}>
            <path
              ref={ref => (pathRefs.current[trajPath.traj.id] = ref)}
              d={trajPath.d}
              stroke="green"
              opacity="0.8"
              strokeWidth={footprint * 0.8}
              fill="none"
              pathLength={1}
              strokeDasharray={1}
              strokeDashoffset={1}
            />
            <path
              d={trajPath.d}
              stroke="green"
              opacity="0.4"
              strokeWidth={footprint * 0.8}
              fill="none"
            />
          </g>
        ))}
      </svg>
    </SVGOverlay>
  );
}

interface TrajectoryPath {
  traj: Trajectory;
  d: string;
  segOffsets: number[];
}
