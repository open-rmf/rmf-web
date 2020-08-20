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

  async robotTrajectoryColor(name: string, model: string): Promise<string> {
    let color = this._pathColorCache[name];
    if (!color) {
      const modelHash = new Uint16Array(await _hash(model));
      const nameHash = new Uint16Array(await _hash(name));
      color = ColorManager._gePathColor(modelHash[0], nameHash[0]);
      this._pathColorCache[name] = color;
    }
    return color;
  }

  async robotImageIconColor(path: string, name: string): Promise<string> {
    let color = this._robotColorCache[name];
    if (!color) {
      const imgHolder = new Image(400, 400); // FIXME -> hardcoded dimensions
      imgHolder.src = path;

      await new Promise((resolve, reject) => {
        imgHolder.onload = () => resolve();
        imgHolder.onerror = err => reject(err);
      });

      const canvas = document.createElement('canvas');
      const context = canvas.getContext('2d');
      context?.drawImage(imgHolder, 0, 0, imgHolder.width, imgHolder.height);
      const data = context?.getImageData(100, 100, 1, 1).data; // FIXME -> need to find betterway to extract point on the image
      if (data) {
        color = `rbg(${data[0]}, ${data[1]}, ${data[2]})`;
        this._robotColorCache[name] = color;
      }
    }
    //FIXME -> returns in rbga format, need to convert to hsl
    return color;
  }

  robotColorFromCache(name: string): string | null {
    return this._robotColorCache[name] ? this._robotColorCache[name] : null;
  }

  pathColorFromCache(name: string): string | null {
    return this._pathColorCache[name] ? this._pathColorCache[name] : null;
  }

  // Gets a light color different than red
  private static _getLightColor(firstNumber: number, secondNumber: number): string {
    // Hue is a degree on the color wheel from 0 to 360. 0 is red, 120 is green, 240 is blue.
    // Add 14 to get a color different than RED
    const hue = 14 + (firstNumber % 360);
    const satlum = secondNumber % 2500;
    // Saturation is a percentage value; 0% means a shade of gray and 100% is the full color.
    const saturation = 50 + (satlum % 50);
    // Lightness is also a percentage; 0% is black, 100% is white.
    const luminance = 25 + satlum / 50;
    return `hsl(${hue}, ${saturation}%, ${luminance}%)`;
  }

  private static _gePathColor(firstNumber: number, secondNumber: number): string {
    // get a range between 90 - 150
    const hue = 90 + (firstNumber % 61);
    // get a range between 20 - 80
    const luminance = 20 + (secondNumber % 61);
    // saturation will stay constant
    return `hsl(${hue}, 100%, ${luminance}%)`;
  }

  private _robotColorCache: Record<string, string> = {};
  private _pathColorCache: Record<string, string> = {};
}
