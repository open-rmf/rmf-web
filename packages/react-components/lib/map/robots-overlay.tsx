import React from 'react';
import * as RmfModels from 'rmf-models';
import { fromRmfCoords, fromRmfYaw } from '../utils/geometry';
import { useAutoScale } from './hooks';
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
  onRobotClick,
  MarkerComponent = RobotMarker,
  bounds,
  ...otherProps
}: RobotsOverlayProps): JSX.Element => {
  const viewBox = viewBoxFromLeafletBounds(bounds);
  const BoundedMarker = React.useMemo(() => bindMarker(MarkerComponent), [MarkerComponent]);
  const scale = useAutoScale(40);
  // TODO: hardcoded because footprint is not available in rmf.
  const footprint = 0.5;

  return (
    <SVGOverlay bounds={bounds} {...otherProps}>
      <svg viewBox={viewBox}>
        {robots.map((robot) => {
          const state = getRobotState(robot.fleet, robot.name);
          if (!state) return;
          const [x, y] = fromRmfCoords([state.location.x, state.location.y]);
          const theta = fromRmfYaw(state.location.yaw);

          return (
            <g key={`${robot.fleet}/${robot.name}`}>
              <BoundedMarker
                robotData={robot}
                color={robot.color}
                iconPath={robot.iconPath}
                onClick={onRobotClick}
                aria-label={robot.name}
                transform={`translate(${x} ${y}) rotate(${theta}) scale(${footprint * scale})`}
              />
              <ManagedNameLabel
                text={robot.name}
                labelTarget={{
                  centerX: x,
                  centerY: y,
                  radius: footprint * scale,
                }}
              />
            </g>
          );
        })}
      </svg>
    </SVGOverlay>
  );
};

export default RobotsOverlay;
