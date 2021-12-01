import { styled } from '@mui/material';
import type { Level } from 'api-client';
import clsx from 'clsx';
import * as L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import React from 'react';
import {
  Map as LMap_,
  MapProps as LMapProps_,
  Pane,
  useLeaflet,
  LeafletContext,
} from 'react-leaflet';
import { EntityManager, EntityManagerContext } from './entity-manager';
import { LabelsPortalContext } from './labels-overlay';
import { SVGOverlay } from './svg-overlay';
import { viewBoxFromLeafletBounds } from './utils';

const classes = {
  map: 'map-root',
};
const StyledLMap_ = styled((props: LMapProps_) => <LMap_ {...props} />)(() => ({
  [`&.${classes.map}`]: {
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

function EntityManagerProvider({
  setLeafletMap,
  children,
}: React.PropsWithChildren<{
  setLeafletMap?: React.Dispatch<React.SetStateAction<LeafletContext>>;
}>) {
  const leaflet = useLeaflet();
  const { current: entityManager } = React.useRef(new EntityManager());

  React.useEffect(() => {
    if (!leaflet.map) return;
    if (setLeafletMap) setLeafletMap(leaflet);
    const listener = () => {
      // TODO: recalculate positions
    };
    leaflet.map.on('zoom', listener);
    return () => {
      leaflet.map && leaflet.map.off('zoom', listener);
    };
  }, [leaflet, leaflet.map, setLeafletMap]);

  return entityManager ? (
    <EntityManagerContext.Provider value={entityManager}>{children}</EntityManagerContext.Provider>
  ) : null;
}

export interface LMapProps extends Omit<LMapProps_, 'crs'> {
  ref?: React.Ref<LMap_>;
  setLeafletMap?: React.Dispatch<React.SetStateAction<LeafletContext>>;
}

export const LMap = React.forwardRef(
  ({ className, children, setLeafletMap, ...otherProps }: LMapProps, ref: React.Ref<LMap_>) => {
    const [labelsPortal, setLabelsPortal] = React.useState<SVGSVGElement | null>(null);
    const viewBox = otherProps.bounds ? viewBoxFromLeafletBounds(otherProps.bounds) : undefined;
    return (
      <StyledLMap_
        ref={ref}
        className={clsx(classes.map, className)}
        crs={L.CRS.Simple}
        {...otherProps}
      >
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
      </StyledLMap_>
    );
  },
);
