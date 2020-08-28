import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React, { useMemo } from 'react';
import ColorManager from './colors';
import Robot from './robot';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';
import { viewBoxFromLeafletBounds } from '../../util/css-utils';

export interface DispensersOverlayProps extends SVGOverlayProps {
  colorManager: ColorManager;
  onDispenserClick?(robot: RomiCore.RobotState): void;
  currentFloorName: string;
}

export default function DispensersOverlay(props: DispensersOverlayProps): React.ReactElement {
  const { colorManager, currentFloorName, ...otherProps } = props;
  const viewBox = viewBoxFromLeafletBounds(props.bounds);
  const footprint = 0.5;

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
