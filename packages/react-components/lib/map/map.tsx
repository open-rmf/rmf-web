import { makeStyles } from '@material-ui/core';
import type { Level } from 'api-client';
import clsx from 'clsx';
import * as L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import React from 'react';
import { MapContainer, MapContainerProps, Pane, SVGOverlay, useMap } from 'react-leaflet';
import { EntityManager, EntityManagerContext } from './entity-manager';
import { LabelsPortalContext } from './labels-overlay';
import { viewBoxFromLeafletBounds } from './utils';

const useStyles = makeStyles(() => ({
  map: {
    height: '100%',
    width: '100%',
    margin: 0,
    padding: 0,
  },
}));

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
  const mapInstance = useMap();
  const { current: entityManager } = React.useRef(new EntityManager());

  React.useEffect(() => {
    if (!mapInstance) return;
    const listener = () => {
      // TODO: recalculate positions
    };
    mapInstance.on('zoom', listener);
    return () => {
      mapInstance && mapInstance.off('zoom', listener);
    };
  }, [mapInstance]);

  return entityManager ? (
    <EntityManagerContext.Provider value={entityManager}>{children}</EntityManagerContext.Provider>
  ) : null;
}

export interface LMapProps extends Omit<MapContainerProps, 'crs'> {
  ref?: React.Ref<typeof MapContainer>;
}

//to be removed
function whenMapIsReady(): void {
  alert('Map Ready');
}

export const LMap = React.forwardRef(
  ({ className, children, ...otherProps }: MapContainerProps) => {
    const classes = useStyles();
    const [labelsPortal, setLabelsPortal] = React.useState<SVGSVGElement | null>(null);
    const viewBox = otherProps.bounds ? viewBoxFromLeafletBounds(otherProps.bounds) : '';

    return (
      <MapContainer
        className={clsx(classes.map, className)}
        crs={L.CRS.Simple}
        {...otherProps}
        maxZoom={22}
        whenReady={whenMapIsReady}
      >
        <EntityManagerProvider>
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
