import { makeStyles } from '@material-ui/core';
import Debug from 'debug';
import React from 'react';
import { uniqueId } from '..';

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

export type WaypointMarkerProps = React.SVGProps<SVGGElement>;

export const WaypointMarker = React.forwardRef(
  ({ ...otherProps }: WaypointMarkerProps, ref: React.Ref<SVGGElement>) => {
    debug('render');
    const classes = useStyles();
    const waypointId = React.useMemo(uniqueId, []);
    return (
      <g ref={ref} {...otherProps}>
        <filter id={`waypoint-${waypointId}-shadow`} x="-20%" y="-20%" width="140%" height="140%">
          <feDropShadow dx={-0.1} dy={-0.1} stdDeviation={0.15} floodColor="black" />
        </filter>
        <rect
          className={classes.marker}
          x={-1}
          y={-1}
          width={2}
          height={2}
          fill={'#FFBF00'}
          filter={`url(#waypoint-${waypointId}-shadow)`}
        />
      </g>
    );
  },
);
