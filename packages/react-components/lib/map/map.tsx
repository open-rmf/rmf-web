import { makeStyles } from '@material-ui/core';
import clsx from 'clsx';
import * as L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import React from 'react';
import { Map as LMap_, MapProps as LMapProps_ } from 'react-leaflet';
import * as RmfModels from 'rmf-models';

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

export type LMapProps = Omit<LMapProps_, 'crs'>;

export function LMap({ className, children, ...otherProps }: LMapProps): React.ReactElement {
  const classes = useStyles();

  return (
    <LMap_ className={clsx(classes.map, className)} crs={L.CRS.Simple} {...otherProps}>
      {children}
    </LMap_>
  );
}
