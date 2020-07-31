import { radiansToDegrees, transformMiddleCoordsOfRectToSVGBeginPoint } from "../calculation-helpers"

test('Convert correctly radians to degrees', () => {
    expect(radiansToDegrees(1)).toBeCloseTo(57.295);
    expect(radiansToDegrees(1.5708)).toBeCloseTo(90);
    expect(radiansToDegrees(-8)).toBeCloseTo(-458.366);
    expect(radiansToDegrees(0)).toBeCloseTo(0);
})


test('Translate middle coords of a rect to svg begin points correctly', () => {
    expect(transformMiddleCoordsOfRectToSVGBeginPoint(1, -1, 2, 2)).toEqual({ x: 0, y: 0 });
    expect(transformMiddleCoordsOfRectToSVGBeginPoint(1, 1, 2, 2)).toEqual({ x: 0, y: 2 });
});