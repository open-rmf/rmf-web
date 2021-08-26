import React from 'react';
import { Place } from '../place';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';
import { viewBoxFromLeafletBounds } from './utils';
import { WaypointMarker as WaypointMarker_ } from './waypoint-marker';

const WaypointMarker = React.memo(WaypointMarker_);

export interface WaypointsOverlayProps extends SVGOverlayProps {
  waypoints: Place[];
}

export const WaypointsOverlay = ({
  waypoints,
  bounds,
  ...otherProps
}: WaypointsOverlayProps): JSX.Element => {
  const viewBox = viewBoxFromLeafletBounds(bounds);
  // Set the size of the waypoint. At least for now we don't want for this to change. We left this here in case we want for this to change in the future.
  const size = 0.1;

  return (
    <SVGOverlay bounds={bounds} {...otherProps}>
      <svg viewBox={viewBox}>
        {waypoints.map((waypoint, idx) => (
          <WaypointMarker
            key={idx}
            waypoint={waypoint.vertex}
            size={size}
            aria-label={waypoint.vertex.name}
          />
        ))}
      </svg>
    </SVGOverlay>
  );
};

export default WaypointsOverlay;
