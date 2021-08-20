import type { AffineImage } from 'api-client';
import L from 'leaflet';
import * as RmfModels from 'rmf-models';

export function viewBoxFromLeafletBounds(bounds: L.LatLngBoundsExpression): string {
  const lbounds = bounds instanceof L.LatLngBounds ? bounds : new L.LatLngBounds(bounds);
  const width = lbounds.getEast() - lbounds.getWest();
  const height = lbounds.getNorth() - lbounds.getSouth();
  return `${lbounds.getWest()} ${lbounds.getNorth()} ${width} ${height}`;
}

export function affineImageBounds(
  image: RmfModels.AffineImage,
  width: number,
  height: number,
): L.LatLngBoundsExpression {
  // FIXME: This assumes that the origin is at the top left.
  // RMF does not provide enough information to determine the origin. We need at least 2
  // points to draw a rectangle, but RMF only provides x, y, width. height. Width and height
  // cannot find the end points because they are absolute values.
  return [
    [image.y_offset, image.x_offset],
    [-(height * image.scale + image.y_offset), width * image.scale + image.x_offset],
  ];
}

export function loadImage(image: AffineImage): Promise<HTMLImageElement> {
  const imageElement = new Image();
  const imageUrl = image.data;
  imageElement.src = imageUrl;

  return new Promise((res) => {
    const listener = () => {
      imageElement.removeEventListener('load', listener);
      res(imageElement);
    };
    imageElement.addEventListener('load', listener);
  });
}
