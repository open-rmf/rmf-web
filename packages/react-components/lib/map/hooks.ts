import React from 'react';
import { useLeaflet } from 'react-leaflet';
import { DefaultMarkerActualSizeMinZoom } from './constants';

/**
 * Get a scaling factor based on the current zoom level.
 * @param factor
 * @param threshold minimal zoom level for the scale to be applied.
 * @returns
 */
export function useAutoScale(factor: number, threshold = DefaultMarkerActualSizeMinZoom): number {
  const leaflet = useLeaflet();

  const getScale = React.useCallback(() => {
    if (!leaflet.map || leaflet.map.getZoom() >= threshold) return 1;
    return factor / 2 ** leaflet.map.getZoom();
  }, [leaflet.map, threshold, factor]);

  const [scale, setScale] = React.useState(getScale);

  React.useEffect(() => {
    if (!leaflet.map) return;
    leaflet.map.on('zoom', () => setScale(getScale()));
  }, [leaflet.map, getScale]);

  return scale;
}
