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
}

export default function RobotsOverlay(props: RobotsOverlayProps): React.ReactElement {
  const { robots, colorManager, onRobotClick, ...otherProps } = props;

  const bounds =
    props.bounds instanceof L.LatLngBounds ? props.bounds : new L.LatLngBounds(props.bounds);
  const width = bounds.getEast() - bounds.getWest();
  const height = bounds.getNorth() - bounds.getSouth();
  const viewBox = `0 0 ${width} ${height}`;
  const footprint = 0.5; // FIXME: hardcode for now, footprint data not available.
  return (
    <SVGOverlay {...otherProps}>
      <svg viewBox={viewBox}>
        {robots.map(robot => {
          return (
            <Robot
              key={robot.name}
              robot={robot}
              footprint={footprint}
              colorManager={colorManager}
              onClick={(_, robot_) => onRobotClick && onRobotClick(robot_)}
            />
          );
        })}
      </svg>
    </SVGOverlay>
  );
}
