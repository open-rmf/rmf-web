import React from 'react';
import { useLeaflet } from 'react-leaflet';
import * as RmfModels from 'rmf-models';
import { fromRmfCoords, fromRmfYaw } from '../utils/geometry';
import { DefaultMarkerActualSizeMinZoom } from './constants';
import { ManagedNameLabel } from './label-manager';
import { RobotMarker as RobotMarker_, RobotMarkerProps } from './robot-marker';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';
import { viewBoxFromLeafletBounds } from './utils';

const RobotMarker = React.memo(RobotMarker_);

export interface RobotData {
  fleet: string;
  name: string;
  model: string;
  footprint: number;
  color: string;
  inConflict?: boolean;
  iconPath?: string;
}

interface BoundedMarkerProps extends Omit<RobotMarkerProps, 'onClick'> {
  robotData: RobotData;
  onClick?: (ev: React.MouseEvent, fleet: string, robot: string) => void;
}

/**
 * Bind a marker to include the fleet and robot name in the click event.
 * This is needed to avoid re-rendering all markers when only one of them changes.
 *
 * Also allows them to track labels.
 */
function bindMarker(MarkerComponent: React.ComponentType<RobotMarkerProps>) {
  return ({ robotData, onClick, ...otherProps }: BoundedMarkerProps) => {
    const handleClick = React.useCallback(
      (ev) => onClick && onClick(ev, robotData.fleet, robotData.name),
      [onClick, robotData.fleet, robotData.name],
    );
    return <MarkerComponent onClick={onClick && handleClick} {...otherProps} />;
  };
}

export interface RobotsOverlayProps extends SVGOverlayProps {
  robots: RobotData[];
  getRobotState: (fleet: string, robot: string) => RmfModels.RobotState | null;
  /**
   * The zoom level at which the markers should transition from actual size to fixed size.
   */
  markerActualSizeMinZoom?: number;
  onRobotClick?: (ev: React.MouseEvent, fleet: string, robot: string) => void;
  MarkerComponent?: React.ComponentType<RobotMarkerProps>;
}

export const RobotsOverlay = ({
  robots,
  getRobotState,
  markerActualSizeMinZoom = DefaultMarkerActualSizeMinZoom,
  onRobotClick,
  MarkerComponent = RobotMarker,
  bounds,
  ...otherProps
}: RobotsOverlayProps): JSX.Element => {
  const viewBox = viewBoxFromLeafletBounds(bounds);
  const BoundedMarker = React.useMemo(() => bindMarker(MarkerComponent), [MarkerComponent]);
  const leaflet = useLeaflet();
  const [zoom, setZoom] = React.useState(leaflet.map?.getZoom());

  const getRadius = React.useCallback(
    (footprint): number | null =>
      zoom === undefined || zoom >= markerActualSizeMinZoom ? footprint : 30 / 2 ** zoom,
    [markerActualSizeMinZoom, zoom],
  );

  React.useLayoutEffect(() => {
    const lmap = leaflet.map;
    if (!lmap) return;
    const listener = () => setZoom(lmap.getZoom());
    lmap.on('zoom', listener);
    return () => {
      lmap.off('zoom', listener);
    };
  }, [leaflet.map]);

  const markerProps = new Map<RobotData, RobotMarkerProps>();
  robots.forEach((robot) => {
    const state = getRobotState(robot.fleet, robot.name);
    if (!state) return;
    const [x, y] = fromRmfCoords([state.location.x, state.location.y]);
    const theta = fromRmfYaw(state.location.yaw);
    // TODO: hardcoded because this is not available in rmf.
    const footprint = 0.5;
    const radius = getRadius(footprint);
    if (!radius) return;
    markerProps.set(robot, {
      x,
      y,
      theta,
      radius,
      color: robot.color,
      inConflict: robot.inConflict,
    });
  });

  return (
    <SVGOverlay bounds={bounds} {...otherProps}>
      <svg viewBox={viewBox}>
        {Array.from(markerProps.entries()).map(([r, p]) => (
          <BoundedMarker
            {...p}
            key={r.name}
            robotData={r}
            onClick={onRobotClick}
            aria-label={r.name}
          />
        ))}
        {Array.from(markerProps.entries()).map(([r, p]) => (
          <ManagedNameLabel
            key={r.name}
            text={r.name}
            labelTarget={{
              centerX: p.x,
              centerY: p.y,
              radius: p.radius,
            }}
          />
        ))}
      </svg>
    </SVGOverlay>
  );
};

export default RobotsOverlay;
