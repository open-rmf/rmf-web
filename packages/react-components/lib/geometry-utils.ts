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
 * Converts a SVG rotation (in degrees) to RMF yaw (in radians)
 * @param yaw
 */
export function toRmfYaw(deg: number): number {
  return -((deg / 180) * Math.PI);
}

/**
 * Converts a RMF yaw angle (in radians) to svg (in degrees)
 * @param yaw
 */
export function fromRmfYaw(yaw: number): number {
  return -((yaw / Math.PI) * 180);
}
