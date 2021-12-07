import { LeafletContextInterface, CONTEXT_VERSION, useLeafletContext } from '@react-leaflet/core';
import { useTheme } from '@mui/material';
import type { Level } from 'api-client';
import * as L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import React from 'react';
import { MapContainer, MapContainerProps, Pane, SVGOverlay, useMap } from 'react-leaflet';
import { EntityManager, EntityManagerContext } from './entity-manager';
import { LabelsPortalContext } from './labels-overlay';
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

function EntityManagerProvider({
  setLeafletMap,
  children,
}: React.PropsWithChildren<{
  setLeafletMap?: React.Dispatch<React.SetStateAction<LeafletContextInterface>>;
}>) {
  const mapInstance = useMap();
  const leafletContext = useLeafletContext();
  const { current: entityManager } = React.useRef(new EntityManager());
  React.useEffect(() => {
    if (!mapInstance) return;
    if (setLeafletMap) setLeafletMap({ __version: CONTEXT_VERSION, map: mapInstance });
    const listener = () => {
      // TODO: recalculate positions
    };
    mapInstance.on('zoom', listener);
    return () => {
      mapInstance && mapInstance.off('zoom', listener);
    };
  }, [mapInstance, setLeafletMap, leafletContext]);

  return entityManager ? (
    <EntityManagerContext.Provider value={entityManager}>{children}</EntityManagerContext.Provider>
  ) : null;
}

export interface LMapProps extends Omit<MapContainerProps, 'crs'> {
  setLeafletMap?: React.Dispatch<React.SetStateAction<LeafletContextInterface>>;
  leafletMap?: LeafletContextInterface;
}

export const LMap = React.forwardRef(
  ({ className, children, setLeafletMap, ...otherProps }: LMapProps) => {
    const classes = useTheme();
    const [labelsPortal, setLabelsPortal] = React.useState<SVGSVGElement | null>(null);
    const viewBox = otherProps.bounds ? viewBoxFromLeafletBounds(otherProps.bounds) : '';

    return (
      <MapContainer crs={L.CRS.Simple} {...otherProps} maxZoom={22} {...otherProps}>
        <EntityManagerProvider setLeafletMap={setLeafletMap}>
          <LabelsPortalContext.Provider value={labelsPortal}>
            {children}
            {otherProps.bounds && (
              <Pane name="label" style={{ zIndex: 1000 }}>
                <SVGOverlay
                  attributes={{ viewBox: viewBox }}
                  bounds={otherProps.bounds}
                  ref={() => {
                    setLabelsPortal(null);
                  }}
                />
              </Pane>
            )}
          </LabelsPortalContext.Provider>
        </EntityManagerProvider>
      </MapContainer>
    );
  },
);
