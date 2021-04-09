import { makeStyles } from '@material-ui/core';
import * as RmfModels from 'rmf-models';
import Debug from 'debug';
import React from 'react';
import { fromRmfCoords } from '../geometry-utils';

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
    userSelect: 'none',
  },
}));

export interface WaypointMarkerProps extends React.SVGProps<SVGGElement> {
  waypoint: RmfModels.GraphNode;
  /**
   * default: 0.1
   */
  size?: number;
  /**
   * Whether the component should perform a translate transform to put it inline with the position
   * in RMF.
   *
   * default: true
   */
  translate?: boolean;
}

export const WaypointMarker = React.forwardRef(
  (props: WaypointMarkerProps, ref: React.Ref<SVGGElement>) => {
    const { waypoint, size = 0.1, translate = true, ...otherProps } = props;
    debug(`render ${waypoint.name}`);
    const pos = fromRmfCoords([waypoint.x, waypoint.y]);

    const classes = useStyles();
    return (
      <g ref={ref} {...otherProps}>
        <g transform={translate ? `translate(${pos[0]} ${pos[1]})` : undefined}>
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
      </g>
    );
  },
);
