import Big from "big.js";
import { IPose2D } from "../models/Pose2D";

export interface Segment {
  initialPose: number;
  finalPose: number;
  initialVelocity: number;
  finalVelocity: number;
  initialTime: Big;
  finalTime: Big;
}

export interface BigSegment {
  initialPose: Big;
  finalPose: Big;
  initialVelocity: Big;
  finalVelocity: Big;
  initialTime: Big;
  finalTime: Big;
}

export interface CoefficientSet {
  a: Big;
  b: Big;
  c: Big;
  d: Big;
}

export interface Velocity {
  x: number;
  y: number;
  theta: number;
}

export interface Knot {
  pose: IPose2D;
  velocity: Velocity;
  time: Big;
}

export interface SegmentCoefficients {
  x: CoefficientSet;
  y: CoefficientSet;
  theta: CoefficientSet;
  initialTime: Big;
  finalTime: Big;
}

export function segmentToBig(segment: Segment): BigSegment {
  const { initialTime, finalTime } = segment

  return {
    initialPose: new Big(segment.initialPose),
    finalPose: new Big(segment.finalPose),
    initialVelocity: new Big(segment.initialVelocity),
    finalVelocity: new Big(segment.finalVelocity),
    initialTime,
    finalTime
  }
}

export function segmentToCoefficientSet(segment: Segment): CoefficientSet {
  const {
    initialPose: x0,
    finalPose: x1,
    initialVelocity: v0,
    finalVelocity: v1,
    initialTime,
    finalTime,
  } = segmentToBig(segment);

  const dt = finalTime.minus(initialTime);
  const w0 = v0.div(dt);
  const w1 = v1.div(dt);

  const a = w1.plus(w0).minus(x1.times(2)).plus(x0.times(2));
  const b = Big(0).minus(w1).minus(w0.times(2)).plus(x1.times(3)).minus(x0.times(3));

  return {
    a,
    b,
    c: w0,
    d: x0,
  };
}

export function assignKnotsToSegment(knot: Knot, nextKnot: Knot, forCoordinate: keyof IPose2D) {
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

export function getInterpolatedTime(initialTime: Big, finalTime: Big, time: Big) {
  return time.minus(initialTime).div(finalTime.minus(initialTime));
}

export function resolveSpline(
  coefficientSet: CoefficientSet,
  interpolatedTime: Big,
  interpolatedTimePow2 = interpolatedTime.pow(2),
  interpolatedTimePow3 = interpolatedTime.pow(3),
): number {
  return parseFloat(coefficientSet.a.times(interpolatedTimePow3)
    .plus(coefficientSet.b.times(interpolatedTimePow2))
    .plus(coefficientSet.c.times(interpolatedTime))
    .plus(coefficientSet.d).toJSON())
}

export function getPositionFromSegmentCoefficientsArray(time: Big, scs: SegmentCoefficients[]): IPose2D | null {
  let sc: SegmentCoefficients | undefined;

  for (sc of scs) {
    if (time >= sc.initialTime && time <= sc.finalTime) {
      break;
    }
  } 

  if (!sc) {
    return null;
  }

  const {
    x: xCoeff,
    y: yCoeff,
    theta: thetaCoeff,
    initialTime,
    finalTime
  } = sc;

  const interpolatedTime = getInterpolatedTime(initialTime, finalTime, time);
  const interpolatedTimePow2 = interpolatedTime.pow(2);
  const interpolatedTimePow3 = interpolatedTime.pow(3);

  return {
    x: resolveSpline(xCoeff, interpolatedTime, interpolatedTimePow2, interpolatedTimePow3),
    y: resolveSpline(yCoeff, interpolatedTime, interpolatedTimePow2, interpolatedTimePow3),
    theta: resolveSpline(thetaCoeff, interpolatedTime, interpolatedTimePow2, interpolatedTimePow3),
  };
}
