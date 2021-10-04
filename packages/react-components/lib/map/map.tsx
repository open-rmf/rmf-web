import { makeStyles } from '@material-ui/core';
import clsx from 'clsx';
import * as L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import React from 'react';
import { Map as LMap_, MapProps as LMapProps_, Pane, useLeaflet } from 'react-leaflet';
import * as RmfModels from 'rmf-models';
import { EntityManager, EntityManagerContext } from './entity-manager';
import { LabelsPortalContext } from './labels-overlay';
import { SVGOverlay } from './svg-overlay';
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
  level: RmfModels.Level;
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
  }, [leaflet.map]);

  return entityManager ? (
    <EntityManagerContext.Provider value={entityManager}>{children}</EntityManagerContext.Provider>
  ) : null;
}

export type LMapProps = Omit<LMapProps_, 'crs'>;

export function LMap({ className, children, ...otherProps }: LMapProps): React.ReactElement {
  const classes = useStyles();
  const [labelsPortal, setLabelsPortal] = React.useState<SVGSVGElement | null>(null);
  const viewBox = otherProps.bounds ? viewBoxFromLeafletBounds(otherProps.bounds) : undefined;
  return (
    <LMap_ className={clsx(classes.map, className)} crs={L.CRS.Simple} {...otherProps}>
      <EntityManagerProvider>
        <LabelsPortalContext.Provider value={labelsPortal}>
          {children}
          {otherProps.bounds && (
            <Pane name="label" style={{ zIndex: 1000 }}>
              <SVGOverlay
                ref={(current) => {
                  setLabelsPortal(current?.container || null);
                }}
                viewBox={viewBox}
                bounds={otherProps.bounds}
              />
            </Pane>
          )}
        </LabelsPortalContext.Provider>
      </EntityManagerProvider>
    </LMap_>
  );
}
