import type { AffineImage, Door, Location2D } from 'api-client';
import L from 'leaflet';
import { Door as RmfDoor } from 'rmf-models';
import { fromRmfCoords, fromRmfYaw } from '../utils';

export function viewBoxFromLeafletBounds(bounds: L.LatLngBoundsExpression): string {
  const lbounds = bounds instanceof L.LatLngBounds ? bounds : new L.LatLngBounds(bounds);
  const width = lbounds.getEast() - lbounds.getWest();
  const height = lbounds.getNorth() - lbounds.getSouth();
  return `${lbounds.getWest()} ${lbounds.getNorth()} ${width} ${height}`;
}

export function affineImageBounds(
  image: AffineImage,
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

export function loadAffineImage(image: AffineImage): Promise<HTMLImageElement> {
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

export function getRmfTransform(location: Location2D): string {
  const pos = fromRmfCoords([location.x, location.y]);
  const yaw = (fromRmfYaw(location.yaw) / Math.PI) * 180;
  return `translate(${pos[0]} ${pos[1]}) rotate(${yaw})`;
}

export function getDoorCenter(door: Door): [x: number, y: number] {
  const v1 = [door.v1_x, door.v1_y];
  const v2 = [door.v2_x, door.v2_y];
  switch (door.door_type) {
    case RmfDoor.DOOR_TYPE_SINGLE_SLIDING:
    case RmfDoor.DOOR_TYPE_SINGLE_SWING:
    case RmfDoor.DOOR_TYPE_SINGLE_TELESCOPE:
    case RmfDoor.DOOR_TYPE_DOUBLE_SLIDING:
    case RmfDoor.DOOR_TYPE_DOUBLE_SWING:
    case RmfDoor.DOOR_TYPE_DOUBLE_TELESCOPE:
      return [(v1[0] + v2[0]) / 2, (v2[1] + v1[1]) / 2];
    default:
      throw new Error('unknown door type');
  }
}
