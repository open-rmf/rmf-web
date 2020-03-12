import * as RomiCore from '@osrf/romi-js-core-interfaces';

/**
 * This can be slow so it should be memoed.
 * @param name
 * @param model
 */
export async function computeRobotColor(name: string, model: string): Promise<string> {
  const modelHash = new Uint16Array(await _hash(model));
  const hue = modelHash[0] % 360;
  const nameHash = new Uint16Array(await _hash(name));
  const satlum = nameHash[0] % 2500;
  const saturation = 50 + (satlum % 50);
  const luminance = 25 + satlum / 50;
  return `hsl(${hue}, ${saturation}%, ${luminance}%)`;
}

async function _hash(s: string): Promise<ArrayBuffer> {
  const encoder = new TextEncoder();
  const data = encoder.encode(s);
  return crypto.subtle.digest('SHA-256', data);
}
