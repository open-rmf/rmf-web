import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React, { useMemo } from 'react';
import { viewBoxFromLeafletBounds } from '../../util/css-utils';
import ColorManager from './colors';
import Robot_, { RobotProps } from './robot';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';
import { ResourcesContext } from '../app-contexts';

const debug = Debug('ScheduleVisualizer:RobotsOverlay');

export interface RobotsOverlayProps extends SVGOverlayProps {
  colorManager: ColorManager;
  fleets: readonly RomiCore.FleetState[];
  onRobotClick?(fleet: string, robot: RomiCore.RobotState): void;
  conflictRobotNames: string[][];
  currentFloorName: string;
  RobotComponent?: React.ElementType<RobotProps>;
}

export const RobotsOverlay = React.memo((props: RobotsOverlayProps) => {
  debug('render');

  const {
    fleets,
    colorManager,
    onRobotClick,
    conflictRobotNames,
    currentFloorName,
    RobotComponent,
    ...otherProps
  } = props;
  const Robot = React.useMemo(() => RobotComponent || Robot_, [RobotComponent]);
  const robotResourcesContext = React.useContext(ResourcesContext)?.robots;
  const viewBox = viewBoxFromLeafletBounds(props.bounds);
  const footprint = 0.5;

  function inConflict(robotName: string): boolean {
    // FIXME: hardcode for now, footprint data not available.
    return conflictRobotNames.flat().includes(robotName) ? true : false;
  }

  // Maps every robot to its fleet. Added the model too in case two robots has the same name on different fleets.
  const fleetContainer = useMemo(() => {
    let robotsFleet: Record<string, string> = {};
    fleets.forEach((fleet) => {
      fleet.robots
        .map((robot) => [robot.name, robot.model])
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
    return fleets.flatMap((x) => ({
      fleet: x.name,
      robots: x.robots.filter((r) => r.location.level_name === currentFloorName),
    }));
  }, [fleets, currentFloorName]);

  const handleRobotClick = React.useCallback<Required<RobotProps>['onClick']>(
    (_, fleetName, robot) => onRobotClick && onRobotClick(fleetName, robot),
    [onRobotClick],
  );

  return (
    <SVGOverlay {...otherProps}>
      <svg viewBox={viewBox}>
        {robotsInCurLevel.map(({ fleet, robots }) =>
          robots.map((robot) => (
            <Robot
              key={robot.name}
              robot={robot}
              fleetName={fleetContainer[`${robot.name}_${robot.model}`]}
              footprint={footprint}
              colorManager={colorManager}
              robotHandler={robotResourcesContext}
              onClick={handleRobotClick}
              inConflict={inConflict(robot.name)}
            />
          )),
        )}
      </svg>
    </SVGOverlay>
  );
});

export default RobotsOverlay;
