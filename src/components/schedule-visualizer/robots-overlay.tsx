import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React, { useMemo } from 'react';
import { viewBoxFromLeafletBounds } from '../../util/css-utils';
import ColorManager from './colors';
import Robot_, { RobotProps } from './robot';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';

export interface RobotsOverlayProps extends SVGOverlayProps {
  colorManager: ColorManager;
  fleets: readonly RomiCore.FleetState[];
  onRobotClick?(robot: RomiCore.RobotState): void;
  conflictRobotNames: string[][];
  currentFloorName: string;
  RobotComponent?: React.ElementType<RobotProps>;
}

export default function RobotsOverlay(props: RobotsOverlayProps): React.ReactElement {
  const {
    fleets,
    colorManager,
    onRobotClick,
    conflictRobotNames,
    currentFloorName,
    RobotComponent,
    ...otherProps
  } = props;
  const Robot = RobotComponent || Robot_;
  const viewBox = viewBoxFromLeafletBounds(props.bounds);
  const footprint = 0.5;

  function inConflict(robotName: string): boolean {
    // FIXME: hardcode for now, footprint data not available.
    return conflictRobotNames.flat().includes(robotName) ? true : false;
  }

  // Maps every robot to its fleet. Added the model too in case two robots has the same name on different fleets.
  const fleetContainer = useMemo(() => {
    let robotsFleet: Record<string, string> = {};
    fleets.forEach(fleet => {
      fleet.robots
        .map(robot => [robot.name, robot.model])
        .forEach((robotInfo: string[]) => {
          const robotKey = `${robotInfo[0]}_${robotInfo[1]}`;
          robotsFleet[robotKey] = fleet.name;
        });
    });
    return robotsFleet;
  }, [fleets]);

  const robotsInCurLevel = React.useMemo(() => {
    if (!currentFloorName) {
      return [];
    }
    return fleets.flatMap(x => x.robots.filter(r => r.location.level_name === currentFloorName));
  }, [fleets, currentFloorName]);

  return (
    <SVGOverlay {...otherProps}>
      <svg viewBox={viewBox}>
        {robotsInCurLevel.map(robot => {
          return (
            <Robot
              key={robot.name}
              robot={robot}
              fleetName={fleetContainer[`${robot.name}_${robot.model}`]}
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
