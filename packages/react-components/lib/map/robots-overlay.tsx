import React from 'react';
import { useLeaflet, Marker } from 'react-leaflet';
import * as RmfModels from 'rmf-models';
import { getRmfTransform } from '.';
import { DefaultMarkerActualSizeMinZoom } from './constants';
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
  onClick?: (ev: React.MouseEvent, fleet: string, robot: string) => void;
}

/**
 * Bind a marker to include the fleet and robot name in the click event.
 * This is needed to avoid re-rendering all markers when only one of them changes.
 */
function bindMarker(MarkerComponent: React.ComponentType<RobotMarkerProps>) {
  return ({ onClick, ...otherProps }: BoundedMarkerProps) => {
    const handleClick = React.useCallback(
      (ev) => onClick && onClick(ev, otherProps.fleet, otherProps.name),
      [onClick, otherProps.fleet, otherProps.name],
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
  const getScaleTransform = React.useCallback(() => {
    const zoom = leaflet.map?.getZoom();
    return zoom === undefined
      ? ''
      : zoom >= markerActualSizeMinZoom
      ? `scale(${footprint})`
      : `scale(${30 / 2 ** zoom})`;
  }, [leaflet.map, markerActualSizeMinZoom]);

  // TODO: hardcoded because this is not available in rmf.
  const footprint = 0.5;
  const [scaleTransform, setScaleTransform] = React.useState(getScaleTransform);

  React.useEffect(() => {
    const lmap = leaflet.map;
    if (!lmap) return;
    const listener = () => setScaleTransform(getScaleTransform());
    lmap.on('zoom', listener);
    return () => {
      lmap.off('zoom', listener);
    };
  }, [leaflet.map, getScaleTransform]);

  return (
    <SVGOverlay bounds={bounds} {...otherProps}>
      <svg viewBox={viewBox}>
        {robots
          .map((robot) => {
            const state = getRobotState(robot.fleet, robot.name);
            return state ? (
              <BoundedMarker
                key={robot.name}
                state={state}
                onClick={onRobotClick}
                aria-label={robot.name}
                transform={`${getRmfTransform(state.location)} ${scaleTransform}`}
                {...robot}
              />
            ) : null;
          })
          .filter((x) => x !== null)}
      </svg>
    </SVGOverlay>
  );
};

export default RobotsOverlay;
