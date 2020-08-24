import * as L from 'leaflet';

export function viewBoxFromLeafletBounds(bounds: L.LatLngBoundsExpression): string {
  const lbounds = bounds instanceof L.LatLngBounds ? bounds : new L.LatLngBounds(bounds);
  const width = lbounds.getEast() - lbounds.getWest();
  const height = lbounds.getNorth() - lbounds.getSouth();
  return `0 0 ${width} ${height}`;
}

// colors used outside material ui
export const colorPalette: { [key: string]: string } = {
  unknown: '#cccccc',
};
