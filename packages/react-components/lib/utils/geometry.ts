/* istanbul ignore file */

export function radiansToDegrees(radians: number): number {
  return radians * (180 / Math.PI);
}

/**
 * Transform coords on the middle of a SVG's Rect to top left coords.
 */
export const transformMiddleCoordsOfRectToSVGBeginPoint = (
  x: number,
  y: number,
  width: number,
  depth: number,
): [x: number, y: number] => {
  return [x - width / 2, y + depth / 2];
};

export interface Pose2D {
  x: number;
  y: number;
  theta: number;
}

export interface Segment {
  initialPose: number;
  finalPose: number;
  initialVelocity: number;
  finalVelocity: number;
  initialTime: number;
  finalTime: number;
}

export interface CoefficientSet {
  a: number;
  b: number;
  c: number;
  d: number;
}

export interface Velocity {
  x: number;
  y: number;
  theta: number;
}

export interface Knot {
  pose: Pose2D;
  velocity: Velocity;
  time: number;
}

export interface SegmentCoefficients {
  x: CoefficientSet;
  y: CoefficientSet;
  theta: CoefficientSet;
  initialTime: number;
  finalTime: number;
}

export function segmentToCoefficientSet(segment: Segment): CoefficientSet {
  const {
    initialPose: x0,
    finalPose: x1,
    initialVelocity: v0,
    finalVelocity: v1,
    initialTime,
    finalTime,
  } = segment;

  const dt = finalTime - initialTime;
  const w0 = v0 / dt;
  const w1 = v1 / dt;

  const a = w1 + w0 - x1 * 2 + x0 * 2;
  const b = -w1 - w0 * 2 + x1 * 3 - x0 * 3;

  return {
    a,
    b,
    c: w0,
    d: x0,
  };
}

export function assignKnotsToSegment(
  knot: Knot,
  nextKnot: Knot,
  forCoordinate: keyof Pose2D,
): Segment {
  return {
    initialPose: knot.pose[forCoordinate],
    finalPose: nextKnot.pose[forCoordinate],
    initialVelocity: knot.velocity[forCoordinate],
    finalVelocity: nextKnot.velocity[forCoordinate],
    initialTime: knot.time,
    finalTime: nextKnot.time,
  };
}

export function knotsToSegmentCoefficientsArray(knots: Knot[]): SegmentCoefficients[] {
  const scs: SegmentCoefficients[] = [];

  for (let i = 0; i < knots.length - 1; ++i) {
    const knot = knots[i];
    const nextKnot = knots[i + 1];

    const sc: SegmentCoefficients = {
      x: segmentToCoefficientSet(assignKnotsToSegment(knot, nextKnot, 'x')),
      y: segmentToCoefficientSet(assignKnotsToSegment(knot, nextKnot, 'y')),
      theta: segmentToCoefficientSet(assignKnotsToSegment(knot, nextKnot, 'theta')),
      initialTime: knot.time,
      finalTime: nextKnot.time,
    };

    scs.push(sc);
  }

  return scs;
}

export function getInterpolatedTime(initialTime: number, finalTime: number, time: number): number {
  return (time - initialTime) / (finalTime - initialTime);
}

export function resolveSpline(
  coefficientSet: CoefficientSet,
  interpolatedTime: number,
  interpolatedTimePow2 = interpolatedTime ** 2,
  interpolatedTimePow3 = interpolatedTime ** 3,
): number {
  return (
    coefficientSet.a * interpolatedTimePow3 +
    coefficientSet.b * interpolatedTimePow2 +
    coefficientSet.c * interpolatedTime +
    coefficientSet.d
  );
}

export function getPositionFromSegmentCoefficientsArray(
  time: number,
  scs: SegmentCoefficients[],
): Pose2D | null {
  let sc: SegmentCoefficients | undefined;

  for (sc of scs) {
    if (time >= sc.initialTime && time <= sc.finalTime) {
      break;
    }
  }

  if (!sc) {
    return null;
  }

  const { x: xCoeff, y: yCoeff, theta: thetaCoeff, initialTime, finalTime } = sc;

  const interpolatedTime = getInterpolatedTime(initialTime, finalTime, time);
  const interpolatedTimePow2 = interpolatedTime ** 2;
  const interpolatedTimePow3 = interpolatedTime ** 3;

  return {
    x: resolveSpline(xCoeff, interpolatedTime, interpolatedTimePow2, interpolatedTimePow3),
    y: resolveSpline(yCoeff, interpolatedTime, interpolatedTimePow2, interpolatedTimePow3),
    theta: resolveSpline(thetaCoeff, interpolatedTime, interpolatedTimePow2, interpolatedTimePow3),
  };
}

function bezierHelper(coeffs: CoefficientSet): [number, number, number, number] {
  const a = coeffs.a;
  const b = coeffs.b;
  const c = coeffs.c;
  const d = coeffs.d;
  const p0 = d;
  const p1 = (c + p0 * 3) / 3;
  const p2 = (b - p0 * 3 + p1 * 6) / 3;
  const p3 = a + p0 - p1 * 3 + p2 * 3;

  return [p0, p1, p2, p3];
}

export function bezierControlPoints(
  segmentCoefficients: SegmentCoefficients,
): [x: number, y: number][] {
  const px = bezierHelper(segmentCoefficients.x);
  const py = bezierHelper(segmentCoefficients.y);
  return px.map((x, i) => [x, py[i]]);
}

/**
 * Converts a svg coordinate to RMF.
 * @param pos
 */
export function toRmfCoords(pos: [number, number]): [number, number] {
  return [pos[0], -pos[1]];
}

/**
 * Converts a RMF coordinate to SVG
 * @param pos
 */
export function fromRmfCoords(pos: [number, number]): [number, number] {
  return [pos[0], -pos[1]];
}

/**
 * Converts a SVG rotation (in radians) to RMF yaw (in radians)
 * @param yaw
 */
export function toRmfYaw(yaw: number): number {
  return -yaw;
}

/**
 * Converts a RMF yaw angle (in radians) to svg (in radians)
 * @param yaw
 */
export function fromRmfYaw(yaw: number): number {
  return -yaw;
}
