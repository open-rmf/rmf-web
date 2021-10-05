import React from 'react';
import { Place } from '../place';
import { fromRmfCoords } from '../utils/geometry';
import { useAutoScale } from './hooks';
import { SVGOverlay, SVGOverlayProps } from './svg-overlay';
import { viewBoxFromLeafletBounds } from './utils';
import { WaypointMarker as WaypointMarker_ } from './waypoint-marker';
import { withLabel } from './with-label';

const WaypointMarker = withLabel(WaypointMarker_);

export interface WaypointsOverlayProps extends Omit<SVGOverlayProps, 'viewBox'> {
  waypoints: Place[];
  hideLabels?: boolean;
}

export const WaypointsOverlay = ({
  waypoints,
  hideLabels = false,
  ...otherProps
}: WaypointsOverlayProps): JSX.Element => {
  const viewBox = viewBoxFromLeafletBounds(otherProps.bounds);
  // Set the size of the waypoint. At least for now we don't want for this to change. We left this here in case we want for this to change in the future.
  const size = 0.2;
  const scale = useAutoScale(60);

  return (
    <SVGOverlay viewBox={viewBox} {...otherProps}>
      {waypoints.map((waypoint, idx) => {
        const [x, y] = fromRmfCoords([waypoint.vertex.x, waypoint.vertex.y]);
        return (
          <g key={idx}>
            <WaypointMarker
              cx={x}
              cy={y}
              size={size}
              aria-label={waypoint.vertex.name}
              style={{ transform: `scale(${scale})`, transformOrigin: `${x}px ${y}px` }}
              labelText={waypoint.vertex.name}
              labelSourceX={x}
              labelSourceY={y}
              labelSourceRadius={size / 2}
              hideLabel={hideLabels}
            />
          </g>
        );
      })}
    </SVGOverlay>
  );
};

export default WaypointsOverlay;
