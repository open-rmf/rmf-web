import React from 'react';
import { RobotMarker as RobotMarker_, RobotMarkerProps } from './robot-marker';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';
import { viewBoxFromLeafletBounds } from './utils';

const RobotMarker = React.memo(RobotMarker_);

export type RobotData = Omit<RobotMarkerProps, 'onClick'>;

export interface RobotsOverlayProps extends SVGOverlayProps {
  robots: RobotData[];
  onRobotClick?: RobotMarkerProps['onClick'];
  MarkerComponent?: React.ComponentType<RobotMarkerProps>;
}

export const RobotsOverlay = ({
  robots,
  onRobotClick,
  MarkerComponent = RobotMarker,
  bounds,
  ...otherProps
}: RobotsOverlayProps): JSX.Element => {
  const viewBox = viewBoxFromLeafletBounds(bounds);

  return (
    <SVGOverlay bounds={bounds} {...otherProps}>
      <svg viewBox={viewBox}>
        {robots.map((robot) => (
          <MarkerComponent
            key={robot.name}
            onClick={onRobotClick}
            aria-label={robot.name}
            {...robot}
          />
        ))}
      </svg>
    </SVGOverlay>
  );
};

export default RobotsOverlay;
