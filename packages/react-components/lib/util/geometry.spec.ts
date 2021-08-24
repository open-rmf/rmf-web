import { radiansToDegrees, transformMiddleCoordsOfRectToSVGBeginPoint } from './geometry';

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
