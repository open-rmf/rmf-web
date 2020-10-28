export function radiansToDegrees(radians: number): number {
  return radians * (180 / Math.PI);
}

/**
 * Transform coords on the middle of a SVG's Rect to top left coords.
 */
export function transformMiddleCoordsOfRectToSVGBeginPoint(
  x: number,
  y: number,
  width: number,
  depth: number,
): [number, number] {
  return [x - width / 2, y + depth / 2];
}
