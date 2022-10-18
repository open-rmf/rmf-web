import React from 'react';
import { useMap } from 'react-leaflet';
import { DefaultMarkerActualSizeMinZoom } from './constants';

/**
 * Get a scaling factor based on the current zoom level.
 * @param factor
 * @param threshold minimal zoom level for the scale to be applied.
 * @returns
 */
export function useAutoScale(factor: number, threshold = DefaultMarkerActualSizeMinZoom): number {
  const leaflet = useMap();

  const getScale = React.useCallback(() => {
    if (!leaflet || leaflet.getZoom() >= threshold) return 1;
    return factor / 2 ** leaflet.getZoom();
  }, [leaflet, threshold, factor]);

  const [scale, setScale] = React.useState(getScale);

  React.useEffect(() => {
    if (!leaflet) return;
    leaflet.on('zoom', () => setScale(getScale()));
  }, [leaflet, getScale]);

  return scale;
}
