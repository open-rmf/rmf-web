import Debug from 'debug';
import React from 'react';
import { WaypointMarker as WaypointMarker_ } from 'react-components';
import * as RmfModels from 'rmf-models';
import { viewBoxFromLeafletBounds } from '../../util/css-utils';
import { PlacesContext } from '../rmf-app';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';

const debug = Debug('ScheduleVisualier:WaypointsOverlay');
const WaypointMarker = React.memo(WaypointMarker_);

export interface WaypointsOverlayProps extends SVGOverlayProps {
  currentLevel: RmfModels.Level;
}

export const WaypointsOverlay = (props: WaypointsOverlayProps) => {
  debug('render');

  const { currentLevel, ...otherProps } = props;
  const viewBox = viewBoxFromLeafletBounds(props.bounds);
  // Set the size of the waypoint. At least for now we don't want for this to change. We left this here in case we want for this to change in the future.
  const size = 0.1;
  const places = React.useContext(PlacesContext);
  const waypoints = places.filter((p) => p.level === currentLevel.name).map((p) => p.vertex);

  return (
    <SVGOverlay {...otherProps}>
      <svg viewBox={viewBox}>
        {waypoints.map((waypoint, idx) => (
          <WaypointMarker key={idx} waypoint={waypoint} size={size} data-testid="waypointMarker" />
        ))}
      </svg>
    </SVGOverlay>
  );
};

export default WaypointsOverlay;
