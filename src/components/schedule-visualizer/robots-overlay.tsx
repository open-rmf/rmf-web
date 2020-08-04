import * as RomiCore from '@osrf/romi-js-core-interfaces';
import * as L from 'leaflet';
import React, { useMemo } from 'react';
import ColorManager from './colors';
import Robot from './robot';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';

export interface RobotsOverlayProps extends SVGOverlayProps {
  robots: readonly RomiCore.RobotState[];
  colorManager: ColorManager;
  fleets: readonly RomiCore.FleetState[];
  onRobotClick?(robot: RomiCore.RobotState): void;
  conflictRobotNames: string[][];
}

export default function RobotsOverlay(props: RobotsOverlayProps): React.ReactElement {
  const { robots, fleets, colorManager, onRobotClick, conflictRobotNames, ...otherProps } = props;

  const bounds =
    props.bounds instanceof L.LatLngBounds ? props.bounds : new L.LatLngBounds(props.bounds);
  const width = bounds.getEast() - bounds.getWest();
  const height = bounds.getNorth() - bounds.getSouth();
  const viewBox = `0 0 ${width} ${height}`;
  const footprint = 0.5;

  function inConflict(robotName: string): boolean {
    // FIXME: hardcode for now, footprint data not available.
    return conflictRobotNames.flat().includes(robotName) ? true : false;
  }

  const fleetContainer = useMemo(() => {
    let robotsFleet: Record<string, string> = {};
    fleets.forEach(fleet => {
      fleet.robots
        .map(robot => robot.name)
        .forEach((robotName: string) => {
          robotsFleet[robotName] = fleet.name;
        });
    });
    return robotsFleet;
  }, [fleets]);

  return (
    <SVGOverlay {...otherProps}>
      <svg viewBox={viewBox}>
        {robots.map(robot => {
          return (
            <Robot
              key={robot.name}
              robot={robot}
              fleetName={fleetContainer[robot.name]}
              footprint={footprint}
              colorManager={colorManager}
              onClick={(_, robot_) => onRobotClick && onRobotClick(robot_)}
              inConflict={inConflict(robot.name)}
            />
          );
        })}
      </svg>
    </SVGOverlay>
  );
}
