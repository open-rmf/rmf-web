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

export default class ColorManager {
  async robotColor(name: string, model: string): Promise<string> {
    const key = this._robotKey(name, model);
    let color = this._robotColorCache[key];
    if (!color) {
      const modelHash = new Uint16Array(await _hash(model));
      const nameHash = new Uint16Array(await _hash(name));
      color = ColorManager._getLightColor(modelHash[0], nameHash[0]);
      this._robotColorCache[key] = color;
      this._robotColorCache[name] = color;
    }
    return color;
  }

  robotColorFromCache(name: string, model?: string): string | null {
    const color = !!model ? this._robotColorCache[this._robotKey(name, model)] : this._robotColorCache[name];
    return color ? color : null;
  }

  private static _getLightColor(firstNumber: number, secondNumber: number): string {
    const hue = firstNumber % 360;
    const satlum = secondNumber % 2500;
    const saturation = 50 + (satlum % 50);
    const luminance = 25 + satlum / 50;
    return `hsl(${hue}, ${saturation}%, ${luminance}%)`;
  }

  private _robotColorCache: Record<string, string> = {};

  private _robotKey(name: string, model: string) {
    return `${name}__${model}`;
  }
}
