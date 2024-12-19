import { describe, expect, it } from 'vitest';

import {
  bezierControlPoints,
  CoefficientSet,
  fromRmfCoords,
  fromRmfYaw,
  getPositionFromSegmentCoefficientsArray,
  Pose2D,
  radiansToDegrees,
  Segment,
  SegmentCoefficients,
  segmentToCoefficientSet,
  toRmfCoords,
  toRmfYaw,
  transformMiddleCoordsOfRectToSVGBeginPoint,
} from './geometry';

it('Convert correctly radians to degrees', () => {
  expect(radiansToDegrees(1)).toBeCloseTo(57.295);
  expect(radiansToDegrees(1.5708)).toBeCloseTo(90);
  expect(radiansToDegrees(-8)).toBeCloseTo(-458.366);
  expect(radiansToDegrees(0)).toBeCloseTo(0);
});

it('Translate middle coords of a rect to svg begin points correctly', () => {
  expect(transformMiddleCoordsOfRectToSVGBeginPoint(1, -1, 2, 2)).toEqual([0, 0]);
  expect(transformMiddleCoordsOfRectToSVGBeginPoint(1, 1, 2, 2)).toEqual([0, 2]);
});

it('fromRmfCoords', () => {
  const result = fromRmfCoords([1, 1]);
  expect(result[0]).toBe(1);
  expect(result[1]).toBe(-1);
});

it('toRmfCoords', () => {
  const result = toRmfCoords([1, 1]);
  expect(result[0]).toBe(1);
  expect(result[1]).toBe(-1);
});

it('fromRmfYaw', () => {
  const result = fromRmfYaw(Math.PI / 4);
  expect(result).toBeCloseTo(-Math.PI / 4);
});

it('toRmfYaw', () => {
  const result = toRmfYaw(Math.PI / 4);
  expect(result).toBeCloseTo(-Math.PI / 4);
});

it('snapshot - getPositionFromSegmentCoefficientsArray', () => {
  const pos = getPositionFromSegmentCoefficientsArray(5, [
    {
      initialTime: 0,
      finalTime: 10,
      theta: { a: 0, b: 0, c: 0, d: 0 },
      x: { a: 1, b: 1, c: 1, d: 1 },
      y: { a: 2, b: 2, c: 2, d: 2 },
    },
  ])!;
  expect(pos.x).toBeCloseTo(1.875);
  expect(pos.y).toBeCloseTo(3.75);
  expect(pos.theta).toBeCloseTo(0);
});

it('snapshot - bezierControlPoints', () => {
  const points = bezierControlPoints({
    initialTime: 0,
    finalTime: 10,
    theta: { a: 0, b: 0, c: 0, d: 0 },
    x: { a: 1, b: 1, c: 1, d: 1 },
    y: { a: 2, b: 2, c: 2, d: 2 },
  });
  expect(points[0][0]).toBeCloseTo(1);
  expect(points[0][1]).toBeCloseTo(2);
  expect(points[1][0]).toBeCloseTo(1.333);
  expect(points[1][1]).toBeCloseTo(2.667);
  expect(points[2][0]).toBeCloseTo(2);
  expect(points[2][1]).toBeCloseTo(4);
  expect(points[3][0]).toBeCloseTo(4);
  expect(points[3][1]).toBeCloseTo(8);
});

describe('segmentToCoefficientSet', () => {
  it('should calculate coefficients correctly for a basic segment', () => {
    const segment: Segment = {
      initialPose: 0,
      finalPose: 1,
      initialVelocity: 0,
      finalVelocity: 0,
      initialTime: 0,
      finalTime: 1,
    };
    const expected: CoefficientSet = {
      a: -2,
      b: 3,
      c: 0,
      d: 0,
    };

    const result = segmentToCoefficientSet(segment);
    expect(result).toEqual(expected);
  });

  it('should handle non-zero initial and final velocities', () => {
    const segment: Segment = {
      initialPose: 1,
      finalPose: 5,
      initialVelocity: 1,
      finalVelocity: 2,
      initialTime: 0,
      finalTime: 2,
    };
    const expected: CoefficientSet = {
      a: -6.5, // (2/2) + (1/2) - (5*2) + (1*2) = 1 + 0.5 - 10 + 2 = -6.5
      b: 10, // -(2/2) - (2*(1/2)) + (5*3) - (1*3) = -1 - 1 + 15 - 3 = 10
      c: 0.5,
      d: 1,
    };

    const result = segmentToCoefficientSet(segment);
    expect(result).toEqual(expected);
  });

  it('should handle non-zero initial time', () => {
    const segment: Segment = {
      initialPose: 2,
      finalPose: 4,
      initialVelocity: 1,
      finalVelocity: 1,
      initialTime: 1,
      finalTime: 3,
    };
    const expected: CoefficientSet = {
      a: -3, // (1/2) + (1/2) - (4*2) + (2*2) = 1 - 8 + 4 = -3
      b: 4.5, // -(1/2) - (2*(1/2)) + (4*3) - (2*3) = -0.5 - 1 + 12 - 6 = 4.5
      c: 0.5,
      d: 2,
    };

    const result = segmentToCoefficientSet(segment);
    expect(result).toEqual(expected);
  });
});

describe('getPositionFromSegmentCoefficientsArray', () => {
  it('should calculate the correct position for a time within the first segment', () => {
    const scs: SegmentCoefficients[] = [
      {
        x: { a: -2, b: 3, c: 0, d: 0 },
        y: { a: 1, b: -2, c: 1, d: 0 },
        theta: { a: 0, b: 0, c: 0, d: 0 },
        initialTime: 0,
        finalTime: 1,
      },
      {
        x: { a: 1, b: -1.5, c: 0.5, d: 1 },
        y: { a: -1, b: 2, c: 0, d: 0 },
        theta: { a: 0, b: 0, c: 0, d: 0 },
        initialTime: 1,
        finalTime: 2,
      },
    ];
    const time = 0.5;
    const expected: Pose2D = {
      x: 0.5, // -2(0.5)^3 + 3(0.5)^2 + 0(0.5) + 0 = -0.25 + 0.75 = 0.5
      y: 0.125, // 1(0.5)^3 - 2(0.5)^2 + 1(0.5) + 0 = 0.125 - 0.5 + 0.5 = 0.125
      theta: 0,
    };

    const result = getPositionFromSegmentCoefficientsArray(time, scs);
    expect(result).toEqual(expected);
  });

  it('should calculate the correct position for a time at the boundary of two segments', () => {
    const scs: SegmentCoefficients[] = [
      {
        x: { a: -2, b: 3, c: 0, d: 0 },
        y: { a: 1, b: -2, c: 1, d: 0 },
        theta: { a: 0, b: 0, c: 0, d: 0 },
        initialTime: 0,
        finalTime: 1,
      },
      {
        x: { a: 1, b: -1.5, c: 0.5, d: 1 },
        y: { a: -1, b: 2, c: 0, d: 0 },
        theta: { a: 0, b: 0, c: 0, d: 0 },
        initialTime: 1,
        finalTime: 2,
      },
    ];
    const time = 1;
    const expected: Pose2D = {
      x: 1, // -2(1)^3 + 3(1)^2 + 0(1) + 0 = -2 + 3 = 1
      y: 0, // 1(1)^3 - 2(1)^2 + 1(1) + 0 = 1 - 2 + 1 = 0
      theta: 0,
    };

    const result = getPositionFromSegmentCoefficientsArray(time, scs);
    expect(result).toEqual(expected);
  });

  it('should calculate the correct position for different coefficient values', () => {
    const scs: SegmentCoefficients[] = [
      {
        x: { a: 0, b: 0, c: 0, d: 5 }, // Constant x
        y: { a: 0, b: 0, c: 2, d: 0 }, // Linear y
        theta: { a: -1, b: 3, c: -2, d: 1 }, // Cubic theta
        initialTime: 0,
        finalTime: 1,
      },
    ];
    const time = 0.5;
    const expected: Pose2D = {
      x: 5,
      y: 1,
      theta: 0.625,
    };

    const result = getPositionFromSegmentCoefficientsArray(time, scs);
    expect(result).toEqual(expected);
  });
});
