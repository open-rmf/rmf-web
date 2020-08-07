import { radiansToDegrees } from '../angle-calculation';

test('Convert correctly radians to degrees', () => {
  expect(radiansToDegrees(1)).toBeCloseTo(57.295);
  expect(radiansToDegrees(1.5708)).toBeCloseTo(90);
  expect(radiansToDegrees(-8)).toBeCloseTo(-458.366);
  expect(radiansToDegrees(0)).toBeCloseTo(0);
});
