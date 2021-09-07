import { bezierControlPoints, Knot, knotsToSegmentCoefficientsArray } from '../utils/geometry';

export type RawVelocity = [x: number, y: number, theta: number];

export type RawPose2D = [x: number, y: number, theta: number];

export interface RawKnot {
  t: number; // milliseconds
  v: RawVelocity;
  x: RawPose2D;
}

export interface Trajectory {
  id: number;
  shape: string;
  dimensions: number;
  segments: RawKnot[];
  robot_name: string;
  fleet_name: string;
  map_name: string;
}

function rawKnotsToKnots(rawKnots: RawKnot[]): Knot[] {
  const knots: Knot[] = [];

  for (const rawKnot of rawKnots) {
    const [poseX, poseY, poseTheta] = rawKnot.x;
    const [velocityX, velocityY, velocityTheta] = rawKnot.v;
    knots.push({
      pose: {
        x: poseX,
        y: poseY,
        theta: poseTheta,
      },
      velocity: {
        x: velocityX,
        y: velocityY,
        theta: velocityTheta,
      },
      time: rawKnot.t,
    });
  }

  return knots;
}

export interface TrajectoryPath {
  d: string;
  segOffsets: number[];
}

export function trajectoryPath(trajectorySegments: RawKnot[]): TrajectoryPath {
  const knots = rawKnotsToKnots(trajectorySegments);
  const coeff = knotsToSegmentCoefficientsArray(knots);
  const bezierSplines = coeff.map(bezierControlPoints);

  const totalDuration = knots[knots.length - 1].time - knots[0].time;
  const segOffsets = knots.map((k) => (k.time - knots[0].time) / totalDuration);

  let d = `M ${bezierSplines[0][0][0]} ${-bezierSplines[0][0][1]} C `;
  bezierSplines.map(
    (bzCurves) =>
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
