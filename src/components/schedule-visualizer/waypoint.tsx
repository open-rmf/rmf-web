import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';

export interface WaypointProps {
  waypoint: RomiCore.GraphNode;
  size: number;
}

const Waypoint = (props: WaypointProps): React.ReactElement => {
  const { waypoint, size } = props;
  const classes = useStyles();
  return (
    <g transform={`translate(${waypoint.x} ${-waypoint.y})`}>
      <filter id={`${waypoint.name}-shadow`} x="-20%" y="-20%" width="140%" height="140%">
        <feDropShadow
          dx={-size * 0.1}
          dy={-size * 0.1}
          stdDeviation={size * 0.15}
          floodColor="black"
        />
      </filter>
      <rect
        className={classes.waypointMarker}
        x={-size}
        y={-size}
        width={size * 2}
        height={size * 2}
        fill={'#FFBF00'}
        filter={`url(#${waypoint.name}-shadow)`}
      />
      <text id="wayportName" y={size * 2.5} className={classes.waypointText}>
        {waypoint.name}
      </text>
    </g>
  );
};

export default Waypoint;

const useStyles = makeStyles(() => ({
  waypointMarker: {
    pointerEvents: 'none',
  },
  waypointText: {
    dominantBaseline: 'central',
    textAnchor: 'middle',
    fontSize: '0.25px',
    fontWeight: 'bold',
    fill: 'white',
    /* 1 pixel black shadow to left, top, right and bottom */
    textShadow: '-1px 0 black, 0 1px black, 1px 0 black, 0 -1px black',
  },
}));
