import { makeStyles } from '@material-ui/core';
import Debug from 'debug';
import React from 'react';
import { uniqueId } from '../utils';

const debug = Debug('Map:WaypointMarker');

const useStyles = makeStyles(() => ({
  marker: {
    pointerEvents: 'none',
  },
  text: {
    dominantBaseline: 'central',
    textAnchor: 'middle',
    fontSize: '0.25px',
    fontWeight: 'bold',
    fill: 'white',
    /* 1 pixel black shadow to left, top, right and bottom */
    textShadow: '-1px 0 black, 0 1px black, 1px 0 black, 0 -1px black',
    userSelect: 'none',
  },
}));

export interface WaypointMarkerProps extends React.PropsWithRef<React.SVGProps<SVGGElement>> {
  cx: number;
  cy: number;
  size: number;
}

export const WaypointMarker = React.forwardRef(
  ({ cx, cy, size, ...otherProps }: WaypointMarkerProps, ref: React.Ref<SVGGElement>) => {
    debug('render');
    const classes = useStyles();
    const waypointId = React.useMemo(uniqueId, []);
    return (
      <g ref={ref} {...otherProps}>
        <defs>
          <filter
            id={`waypoint-${waypointId}-shadow`}
            x="-20%"
            y="-20%"
            width="140%"
            height="140%"
            filterUnits="userSpaceOnUse"
          >
            <feDropShadow
              dx={-0.05 * size}
              dy={-0.05 * size}
              stdDeviation={0.15 * size}
              floodColor="black"
            />
          </filter>
        </defs>
        <rect
          className={classes.marker}
          x={cx - size / 2}
          y={cy - size / 2}
          width={size}
          height={size}
          fill={'#FFBF00'}
          filter={`url(#waypoint-${waypointId}-shadow)`}
        />
      </g>
    );
  },
);
