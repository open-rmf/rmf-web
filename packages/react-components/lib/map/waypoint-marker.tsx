import { styled } from '@mui/material';
import Debug from 'debug';
import React from 'react';
import { uniqueId } from '../utils';

const debug = Debug('Map:WaypointMarker');

const classes = {
  marker: 'waypoint-marker-marker',
  text: 'waypoint-marker-text',
};
const StyledG = styled('g')(() => ({
  [`& .${classes.marker}`]: {
    pointerEvents: 'none',
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
    const waypointId = React.useMemo(uniqueId, []);
    return (
      <StyledG ref={ref} {...otherProps}>
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
      </StyledG>
    );
  },
);
