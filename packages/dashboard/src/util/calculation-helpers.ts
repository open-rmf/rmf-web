export function radiansToDegrees(radians: number) {
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
) => {
  return { x: x - width / 2, y: y + depth / 2 };
};
