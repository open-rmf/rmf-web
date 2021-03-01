import {
  fromRmfCoords,
  fromRmfYaw,
  radiansToDegrees,
  toRmfCoords,
  toRmfYaw,
  transformMiddleCoordsOfRectToSVGBeginPoint,
} from '../lib';

it('radiansToDegrees', () => {
  const result = radiansToDegrees(Math.PI / 4);
  expect(result).toBeCloseTo(45);
});

it('transformMiddleCoordsOfRectToSVGBeginPoint', () => {
  const result = transformMiddleCoordsOfRectToSVGBeginPoint(1, 1, 2, 2);
  expect(result[0]).toBeCloseTo(0);
  expect(result[0]).toBeCloseTo(0);
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
