import { crc32 } from 'crc';
import Vibrant from 'node-vibrant';
import React from 'react';
import { robotHash } from './robots';

function _hash(s: string): number {
  return crc32(s);
}

export class ColorManager {
  readonly conflictHighlight = '#f44336';

  async robotPrimaryColor(
    fleet: string,
    name: string,
    model: string,
    image?: string | HTMLImageElement | Buffer,
  ): Promise<string> {
    const key = robotHash(name, fleet);
    if (this._robotColorCache[key]) {
      return this._robotColorCache[key];
    }

    if (!image) {
      this._robotColorCache[key] = this._robotColorFromId(fleet, name, model);
      return this._robotColorCache[key];
    } else {
      try {
        const palette = await Vibrant.from(image).getSwatches();
        // TODO: remove usage of deprecated method
        const rgb = palette.Vibrant?.getRgb();
        if (rgb) {
          const colorHolder = `rgb(${rgb[0]}, ${rgb[1]}, ${rgb[2]})`;
          this._robotColorCache[key] = colorHolder;
          return colorHolder;
        }
      } catch (e) {
        console.warn(
          `unable to get color from image, falling back to color from id (${(e as Error).message})`,
        );
      }
      return this.robotPrimaryColor(fleet, name, model);
    }
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

  private _robotColorFromId(fleet: string, name: string, model: string): string {
    const modelHash = _hash(model);
    const nameHash = _hash(name);
    return ColorManager._getLightColor(modelHash, nameHash);
  }

  private _robotColorCache: Record<string, string> = {};
}

export default ColorManager;

export const ColorContext = React.createContext(new ColorManager());
