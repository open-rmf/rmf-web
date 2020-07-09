import { radiansToDegrees } from "../angle-calculation"

test('Convert correctly radians to degrees', () => {
    expect(radiansToDegrees(1)).toBe(57.29577951308232);
    expect(radiansToDegrees(1.5708)).toBe(90.00021045914971);
    expect(radiansToDegrees(-8)).toBe(-458.3662361046586);
    expect(radiansToDegrees(0)).toBe(0);
})
