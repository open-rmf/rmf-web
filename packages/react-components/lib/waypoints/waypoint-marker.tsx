import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';

const debug = Debug('Waypoints:WaypointMarker');

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
  },
}));

export interface WaypointMarkerProps {
  waypoint: RomiCore.GraphNode;

  /**
   * default: 0.1
   */
  size?: number;
}

export const WaypointMarker = React.memo(
  React.forwardRef((props: WaypointMarkerProps, ref: React.Ref<SVGGElement>) => {
    const { waypoint, size = 0.1 } = props;
    debug(`render ${waypoint.name}`);

    const classes = useStyles();
    return (
      <g ref={ref} transform={`translate(${waypoint.x} ${-waypoint.y})`}>
        <filter
          id={`waypoint-${waypoint.name}-shadow`}
          x="-20%"
          y="-20%"
          width="140%"
          height="140%"
        >
          <feDropShadow
            dx={-size * 0.1}
            dy={-size * 0.1}
            stdDeviation={size * 0.15}
            floodColor="black"
          />
        </filter>
        <rect
          className={classes.marker}
          x={-size}
          y={-size}
          width={size * 2}
          height={size * 2}
          fill={'#FFBF00'}
          filter={`url(#waypoint-${waypoint.name}-shadow)`}
        />
        <text y={size * 2.5} className={classes.text}>
          {waypoint.name}
        </text>
      </g>
    );
  }),
);
