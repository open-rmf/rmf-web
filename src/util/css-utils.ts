import * as L from 'leaflet';

type Classes = Partial<{ [key: string]: string }>;

export function className<T extends Classes>(classes: T | undefined, name: keyof T): string {
  if (!classes || !classes[name]) {
    return '';
  }
  return classes[name]!;
}

export function viewBoxFromLeafletBounds(bounds: L.LatLngBoundsExpression): string {
  const lbounds = bounds instanceof L.LatLngBounds ? bounds : new L.LatLngBounds(bounds);
  const width = lbounds.getEast() - lbounds.getWest();
  const height = lbounds.getNorth() - lbounds.getSouth();
  return `0 0 ${width} ${height}`;
}
