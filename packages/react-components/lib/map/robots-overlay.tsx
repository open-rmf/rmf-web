import { useTheme } from '@material-ui/core';
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
  state: RmfModels.RobotState;
  inConflict?: boolean;
  /**
   * defaults to theme primary color
   */
  color?: string;
  iconPath?: string;
}

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
  const theme = useTheme();

  return (
    <SVGOverlay bounds={bounds} {...otherProps}>
      <svg viewBox={viewBox}>
        {robots.map((robot) => (
          <MarkerComponent
            key={robot.name}
            color={robot.color || theme.palette.primary.main}
            iconPath={robot.iconPath}
            fleet={robot.fleet}
            name={robot.name}
            model={robot.model}
            footprint={robot.footprint}
            state={robot.state}
            inConflict={robot.inConflict}
            onClick={onRobotClick}
            aria-label={robot.name}
          />
        ))}
      </svg>
    </SVGOverlay>
  );
};

export default RobotsOverlay;
