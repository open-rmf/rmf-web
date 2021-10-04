import React from 'react';
import ReactDOM from 'react-dom';
import { Place } from '../place';
import { fromRmfCoords } from '../utils/geometry';
import { useAutoScale } from './hooks';
import { ScaledNameLabel } from './label-marker';
import { LabelsPortalContext } from './labels-overlay';
import { SVGOverlay, SVGOverlayProps } from './svg-overlay';
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
  const size = 0.2;
  const scale = useAutoScale(60);
  const labelsPortal = React.useContext(LabelsPortalContext);

  return (
    <SVGOverlay bounds={bounds} {...otherProps}>
      <svg viewBox={viewBox}>
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
              />
              {labelsPortal &&
                ReactDOM.createPortal(
                  <ScaledNameLabel
                    text={waypoint.vertex.name}
                    sourceX={x}
                    sourceY={y}
                    sourceRadius={size / 2}
                  />,
                  labelsPortal,
                )}
            </g>
          );
        })}
      </svg>
    </SVGOverlay>
  );
};

export default WaypointsOverlay;
