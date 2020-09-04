import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React, { useMemo } from 'react';
import { viewBoxFromLeafletBounds } from '../../util/css-utils';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';
import Waypoint from './waypoint';

const debug = Debug('ScheduleVisualier:WaypointsOverlay');

export interface WaypointsOverlayProps extends SVGOverlayProps {
  currentLevel: RomiCore.Level;
}

export const WaypointsOverlay = React.memo((props: WaypointsOverlayProps) => {
  debug('render');

  const { currentLevel, ...otherProps } = props;
  const viewBox = viewBoxFromLeafletBounds(props.bounds);
  // Set the size of the waypoint. At least for now we don't want for this to change. We left this here in case we want for this to change in the future.
  const size = 0.1;
  const waypoints = useMemo(() => {
    if (currentLevel.nav_graphs.length === 0) {
      return [];
    }
    return currentLevel.nav_graphs[0].vertices;
  }, [currentLevel]);

  return (
    <SVGOverlay {...otherProps}>
      <svg viewBox={viewBox}>
        {waypoints.map(
          waypoint =>
            !!waypoint.name && <Waypoint key={waypoint.name} waypoint={waypoint} size={size} />,
        )}
      </svg>
    </SVGOverlay>
  );
});

export default WaypointsOverlay;
