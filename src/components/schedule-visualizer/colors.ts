import Vibrant from 'node-vibrant';

async function _hash(s: string): Promise<ArrayBuffer> {
  const encoder = new TextEncoder();
  const data = encoder.encode(s);
  return crypto.subtle.digest('SHA-256', data);
}

export default class ColorManager {
  async robotColor(fleet: string, name: string, model: string): Promise<string> {
    const key = this._robotKey(name, fleet);
    let color = this._robotColorCache[key];
    if (!color) {
      const modelHash = new Uint16Array(await _hash(model));
      const nameHash = new Uint16Array(await _hash(name));
      color = ColorManager._getLightColor(modelHash[0], nameHash[0]);
      this._robotColorCache[key] = color;
    }
    return color;
  }

  async robotPrimaryColor(
    fleet: string,
    name: string,
    model: string,
    image?: string,
  ): Promise<string | undefined> {
    let color: string;
    const key = this._robotKey(name, fleet);
    color = this._pathColorCache[key];

    // set different shades of green for path color
    if (!color) {
      const modelHash = new Uint16Array(await _hash(model));
      const nameHash = new Uint16Array(await _hash(name));
      color = ColorManager._gePathColor(modelHash[0], nameHash[0]);
      this._pathColorCache[key] = color;

      // if image path is provided, proceed to extract colors from image
      if (image) {
        color = this._robotColorCache[key];
        if (!color) {
          const imgHolder = new Image();
          imgHolder.src = image;

          await new Promise((resolve, reject) => {
            imgHolder.onload = () => resolve();
            imgHolder.onerror = err => reject(err);
          });
          return Vibrant.from(imgHolder)
            .getSwatches()
            .then(palette => {
              const rgb = palette.Vibrant?.getRgb();
              if (rgb) {
                const colorHolder = `rgb(${rgb[0]}, ${rgb[1]}, ${rgb[2]})`;
                this._robotColorCache[key] = colorHolder;
                return colorHolder;
              }
            });
        }
      }
    }
    return color;
  }

  robotColorFromCache(fleet: string, name: string): string | null {
    const key = this._robotKey(name, fleet);
    return this._robotColorCache[key] ? this._robotColorCache[key] : null;
  }

  pathColorFromCache(fleet: string, name: string): string | null {
    const key = this._robotKey(name, fleet);
    return this._pathColorCache[key] ? this._pathColorCache[key] : null;
  }

  // Gets a light color different than red
  private static _getLightColor(firstNumber: number, secondNumber: number): string {
    // Hue is a degree on the color wheel from 0 to 360. 0 is red, 120 is green, 240 is blue.
    // keep it within a range of 50-270 to prevent red like colors
    const hue = 50 + (firstNumber % 220);
    const satlum = secondNumber % 2500;
    // Saturation is a percentage value; 0% means a shade of gray and 100% is the full color.
    const saturation = 50 + (satlum % 50);
    // Lightness is also a percentage; 0% is black, 100% is white.
    const luminance = 25 + satlum / 50;
    return `hsl(${hue}, ${saturation}%, ${luminance}%)`;
  }

  private static _gePathColor(firstNumber: number, secondNumber: number): string {
    // get a range between 90 - 150 for a shade of green
    const hue = 90 + (firstNumber % 61);
    // get a range between 20 - 80
    const luminance = 20 + (secondNumber % 61);
    // saturation will stay constant
    return `hsl(${hue}, 100%, ${luminance}%)`;
  }

  private _robotColorCache: Record<string, string> = {};
  private _pathColorCache: Record<string, string> = {};

  private _robotKey(name: string, fleet: string) {
    return `${name}__${fleet}`;
  }
}
