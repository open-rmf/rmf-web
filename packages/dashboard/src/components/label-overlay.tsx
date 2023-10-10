import React from 'react';
import {
  fromRmfCoords,
  Place,
  SVGOverlay,
  SVGOverlayProps,
  viewBoxFromLeafletBounds,
  WaypointLabelMaker,
} from 'react-components';

export interface LabelsOverlayProps extends Omit<SVGOverlayProps, 'viewBox'> {
  waypoints: Place[];
  hideLabels?: boolean;
  pickup?: boolean;
  dropoff?: boolean;
}

export const LabelsOverlay = React.memo(
  ({
    waypoints,
    hideLabels = false,
    pickup,
    dropoff,
    ...otherProps
  }: LabelsOverlayProps): JSX.Element => {
    const viewBox = viewBoxFromLeafletBounds(otherProps.bounds);
    // Set the size of the waypoint. At least for now we don't want for this to change. We left this here in case we want for this to change in the future.
    const size = 0.2;

    return (
      <SVGOverlay viewBox={viewBox} {...otherProps}>
        {waypoints.map((waypoint, idx) => {
          const [x, y] = fromRmfCoords([waypoint.vertex.x, waypoint.vertex.y]);
          return (
            <g key={idx}>
              <WaypointLabelMaker
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
  },
);
