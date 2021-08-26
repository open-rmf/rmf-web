import React from 'react';
import * as RmfModels from 'rmf-models';
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
