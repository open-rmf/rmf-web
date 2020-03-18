import * as L from 'leaflet';
import React from 'react';
import { rawKnotsToKnots, Trajectory } from '../../robot-trajectory-manager';
import { bezierControlPoints, knotsToSegmentCoefficientsArray } from '../../util/cublic-spline';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';

export interface RobotTrajectoriesOverlayProps extends SVGOverlayProps {
  trajs: readonly Trajectory[];
}

export default function RobotTrajectoriesOverlay(
  props: RobotTrajectoriesOverlayProps,
): React.ReactElement {
  const { trajs, ...otherProps } = props;

  const bounds =
    props.bounds instanceof L.LatLngBounds ? props.bounds : new L.LatLngBounds(props.bounds);
  const width = bounds.getEast() - bounds.getWest();
  const height = bounds.getNorth() - bounds.getSouth();
  const viewBox = `0 0 ${width} ${height}`;

  const bezierSplines = trajs.map(traj => {
    const knots = rawKnotsToKnots(traj.segments);
    return knotsToSegmentCoefficientsArray(knots).map(coeff => bezierControlPoints(coeff));
  });

  const ds = bezierSplines.map(bzSpline => {
    let d = `M ${bzSpline[0][0][0]} ${-bzSpline[0][0][1]} C `;
    bzSpline.map(
      bzCurves =>
        (d +=
          `${bzCurves[1][0]} ${-bzCurves[1][1]} ` +
          `${bzCurves[2][0]} ${-bzCurves[2][1]} ` +
          `${bzCurves[3][0]} ${-bzCurves[3][1]} `),
    );
    return d;
  });

  // console.log(bezierSplines);

  return (
    <SVGOverlay {...otherProps}>
      <svg viewBox={viewBox}>
        {ds.map((d, i) => (
          <g key={i}>
            <path d={d} stroke="red" strokeWidth="0.05" fill="none" />
          </g>
        ))}
      </svg>
    </SVGOverlay>
  );
}
