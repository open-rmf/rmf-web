/**
 * TODO: Need door location, (v1_x, v1_y) only defines the location of the hinge, we also need the
 * length and orientationo of the door to draw it on the map.
 */

import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import * as L from 'leaflet';
import React from 'react';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';

const useStyles = makeStyles(() => ({
  doorMarker: {
    cursor: 'pointer',
    pointerEvents: 'auto',
  },
}));

interface DoorsOverlayProps extends SVGOverlayProps {
  doors: readonly RomiCore.Door[];
}

export default function DoorsOverlay(props: DoorsOverlayProps): React.ReactElement {
  const classes = useStyles();
  const bounds =
    props.bounds instanceof L.LatLngBounds ? props.bounds : new L.LatLngBounds(props.bounds);

  const width = bounds.getEast() - bounds.getWest();
  const height = bounds.getNorth() - bounds.getSouth();
  const viewBox = `0 0 ${width} ${height}`;
  const fillColor = 'rgba(0, 0, 0, 0.1)';
  return (
    <SVGOverlay {...props}>
      <svg viewBox={viewBox}>
        {props.doors.map(door => {
          const x = Math.min(door.v1_x, door.v2_x);
          const y = Math.min(door.v1_y, door.v2_y);
          const width = Math.abs(door.v2_x - door.v1_x);
          const height = Math.abs(door.v2_y - door.v1_y);
          return (
            <g key={door.name}>
              <filter id={`${door.name}-shadow`} x="-150%" y="-150%" width="400%" height="400%">
                <feDropShadow dx="0" dy="0" stdDeviation="0.1" floodColor="black" />
              </filter>
              <polygon
                className={classes.doorMarker}
                x={x}
                y={-y}
                width={width}
                height={height}
                fill={fillColor}
                stroke="black"
                strokeWidth="0.03"
                filter={`url(#${door.name}-shadow)`}
              />
            </g>
          );
        })}
      </svg>
    </SVGOverlay>
  );
}
