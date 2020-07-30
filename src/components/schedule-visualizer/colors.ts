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

export default class ColorManager {
  async robotColor(name: string, model: string): Promise<string> {
    let color = this._robotColorCache[name];
    if (!color) {
      const modelHash = new Uint16Array(await _hash(model));
      const nameHash = new Uint16Array(await _hash(name));
      color = ColorManager._getLightColor(modelHash[0], nameHash[0]);
      this._robotColorCache[name] = color;
    }
    return color;
  }

  robotColorFromCache(name: string): string | null {
    return this._robotColorCache[name] ? this._robotColorCache[name] : null;
  }

  // Gets a light color different than red
  private static _getLightColor(firstNumber: number, secondNumber: number): string {
    // Hue is a degree on the color wheel from 0 to 360. 0 is red, 120 is green, 240 is blue. 
    // Add 14 to get a color different than RED
    const hue = 14 + firstNumber % 360;
    const satlum = secondNumber % 2500;
    // Saturation is a percentage value; 0% means a shade of gray and 100% is the full color.
    const saturation = 50 + (satlum % 50);
    // Lightness is also a percentage; 0% is black, 100% is white.
    const luminance = 25 + satlum / 50;
    return `hsl(${hue}, ${saturation}%, ${luminance}%)`;
  }

  private _robotColorCache: Record<string, string> = {};

}