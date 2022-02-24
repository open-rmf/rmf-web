import React from 'react';
import { almostShallowEqual } from '../utils';
import { fromRmfCoords, fromRmfYaw } from '../utils/geometry';
import { RobotState } from 'api-client';
import { useAutoScale } from './hooks';
import { RobotMarker as RobotMarker_, RobotMarkerProps } from './robot-marker';
import { SVGOverlay, SVGOverlayProps } from './svg-overlay';
import { viewBoxFromLeafletBounds } from './utils';
import { withLabel } from './with-label';

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
  fleet: string;
  name: string;
  onClick?: (ev: React.MouseEvent, fleet: string, robot: string) => void;
}

/**
 * Bind a marker to include the fleet and robot name in the click event.
 * This is needed to avoid re-rendering all markers when only one of them changes.
 */
function bindMarker(MarkerComponent: React.ComponentType<RobotMarkerProps>) {
  return ({ fleet, name, onClick, ...otherProps }: BoundedMarkerProps) => {
    const handleClick = React.useCallback(
      (ev) => onClick && onClick(ev, fleet, name),
      [onClick, fleet, name],
    );
    return <MarkerComponent onClick={onClick && handleClick} {...otherProps} />;
  };
}

const RobotMarker = React.memo(withLabel(bindMarker(RobotMarker_)), (prev, next) =>
  almostShallowEqual(prev, next, ['style']),
);

export interface RobotsOverlayProps extends Omit<SVGOverlayProps, 'viewBox'> {
  robots: RobotData[];
  getRobotState: (fleet: string, robot: string) => RobotState | null;
  /**
   * The zoom level at which the markers should transition from actual size to fixed size.
   */
  markerActualSizeMinZoom?: number;
  hideLabels?: boolean;
  onRobotClick?: (ev: React.MouseEvent, fleet: string, robot: string) => void;
}

export const RobotsOverlay = ({
  robots,
  getRobotState,
  hideLabels = false,
  onRobotClick,
  ...otherProps
}: RobotsOverlayProps): JSX.Element => {
  const viewBox = viewBoxFromLeafletBounds(otherProps.bounds);
  const scale = useAutoScale(40);
  // TODO: hardcoded because footprint is not available in rmf.
  const footprint = 0.5;

  return (
    <SVGOverlay viewBox={viewBox} {...otherProps}>
      {robots.map((robot) => {
        const state = getRobotState(robot.fleet, robot.name);
        if (!state) return;
        const [x, y] = state.location
          ? fromRmfCoords([state.location.x, state.location.y])
          : [0, 0];
        const theta = state.location ? fromRmfYaw(state.location.yaw) : 0;
        return (
          <RobotMarker
            key={`${robot.fleet}/${robot.name}`}
            fleet={robot.fleet}
            name={robot.name}
            cx={x}
            cy={y}
            r={footprint}
            color={robot.color}
            iconPath={robot.iconPath}
            onClick={onRobotClick}
            aria-label={robot.name}
            style={{
              transform: `rotate(${theta}rad) scale(${scale})`,
              transformOrigin: `${x}px ${y}px`,
            }}
            labelText={robot.name}
            labelSourceX={x}
            labelSourceY={y}
            labelSourceRadius={footprint * scale}
            hideLabel={hideLabels}
          />
        );
      })}
    </SVGOverlay>
  );
};

export default RobotsOverlay;
