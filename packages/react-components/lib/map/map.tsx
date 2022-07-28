import { useTheme } from '@mui/material';
import type { Level } from 'api-client';
import * as L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import React from 'react';
import { Map as LMap_, MapProps as LMapProps_, Pane, useLeaflet } from 'react-leaflet';
import { EntityManager, EntityManagerContext } from './entity-manager';
import { LabelsPortalContext } from './labels-overlay';
import { SVGOverlay } from './svg-overlay';
import { viewBoxFromLeafletBounds } from './utils';

export interface MapFloorLayer {
  level: Level;
  imageUrl: string;
  bounds: L.LatLngBounds;
}

export function calcMaxBounds(
  mapFloorLayers: readonly MapFloorLayer[],
): L.LatLngBounds | undefined {
  if (!mapFloorLayers.length) {
    return undefined;
  }
  const bounds = new L.LatLngBounds([0, 0], [0, 0]);
  Object.values(mapFloorLayers).forEach((x) => bounds.extend(x.bounds));
  return bounds.pad(0.2);
}

function EntityManagerProvider({ children }: React.PropsWithChildren<{}>) {
  const leaflet = useLeaflet();
  const { current: entityManager } = React.useRef(new EntityManager());
  React.useEffect(() => {
    if (!leaflet.map) return;
    const listener = () => {
      // TODO: recalculate positions
    };
    leaflet.map.on('zoom', listener);
    return () => {
      leaflet.map && leaflet.map.off('zoom', listener);
    };
  }, [leaflet, leaflet.map]);

  return entityManager ? (
    <EntityManagerContext.Provider value={entityManager}>{children}</EntityManagerContext.Provider>
  ) : null;
}

export interface LMapProps extends Omit<LMapProps_, 'crs'> {
  ref?: React.Ref<LMap_>;
}

export const LMap = React.forwardRef(
  ({ children, bounds, ...otherProps }: LMapProps, ref: React.Ref<LMap_>) => {
    const [labelsPortal, setLabelsPortal] = React.useState<SVGSVGElement | null>(null);
    const viewBox = bounds ? viewBoxFromLeafletBounds(bounds) : undefined;
    const theme = useTheme();
    return (
      <LMap_
        ref={ref}
        style={{
          height: '100%',
          width: '100%',
          margin: 0,
          padding: 0,
          backgroundColor: theme.palette.background.default,
        }}
        crs={L.CRS.Simple}
        {...otherProps}
      >
        <EntityManagerProvider>
          <LabelsPortalContext.Provider value={labelsPortal}>
            {children}
            {bounds && (
              <Pane name="label" style={{ zIndex: 1000 }}>
                <SVGOverlay
                  ref={(current) => {
                    setLabelsPortal(current?.container || null);
                  }}
                  viewBox={viewBox}
                  bounds={bounds}
                />
              </Pane>
            )}
          </LabelsPortalContext.Provider>
        </EntityManagerProvider>
      </LMap_>
    );
  },
);
