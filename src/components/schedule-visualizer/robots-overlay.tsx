import * as RomiCore from '@osrf/romi-js-core-interfaces';
import * as L from 'leaflet';
import React from 'react';
import ColorManager from './colors';
import Robot from './robot';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';

export interface RobotsOverlayProps extends SVGOverlayProps {
  robots: readonly RomiCore.RobotState[];
  colorManager: ColorManager;
  onRobotClick?(robot: RomiCore.RobotState): void;
  conflictRobotNames: string[];
}

export default function RobotsOverlay(props: RobotsOverlayProps): React.ReactElement {
  const { robots, colorManager, onRobotClick, conflictRobotNames, ...otherProps } = props;

  const bounds =
    props.bounds instanceof L.LatLngBounds ? props.bounds : new L.LatLngBounds(props.bounds);
  const width = bounds.getEast() - bounds.getWest();
  const height = bounds.getNorth() - bounds.getSouth();
  const viewBox = `0 0 ${width} ${height}`;

  function getRobotFootprint(robotName: string): number {
    // FIXME: hardcode for now, footprint data not available.
    return conflictRobotNames.includes(robotName) ? 0.75 : 0.5;
  }

  return (
    <SVGOverlay {...otherProps}>
      <svg viewBox={viewBox}>
        {robots.map(robot => {
          return (
            <Robot
              key={robot.name}
              robot={robot}
              footprint={getRobotFootprint(robot.name)}
              colorManager={colorManager}
              onClick={(_, robot_) => onRobotClick && onRobotClick(robot_)}
            />
          );
        })}
      </svg>
    </SVGOverlay>
  );
}
