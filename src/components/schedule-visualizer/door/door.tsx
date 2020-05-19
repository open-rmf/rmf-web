import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import { Spinner } from '../spinner';
import SingleHingeDoor from './door-single-hinge';
import DoubleHingeDoor from './door-double-hinge';
import SingleSlideDoor from './door-single-slide';
import DoubleSlideDoor from './door-double-slide';
import DefaultDoor from './door-default';

export enum DoorMode {
  CLOSE,
  PROCESS,
  OPEN,
}

export enum DoorType {
  UNDEFINED,
  SINGLE_SLIDING,
  DOUBLE_SLIDING,
  SINGLE_TELESCOPE,
  DOUBLE_TELESCOPE,
  SINGLE_SWING,
  DOUBLE_SWING,
}

/**
 * currentMode: Current mode of the door. E.g: 0 = DoorMode.CLOSE.
 * door: Door information provided by the map.
 * doorState: Current state of the door.
 * onClick: Action to trigger on click.
 */
export interface DoorContainerProps {
  onClick?(e: React.MouseEvent<SVGGElement>, door: RomiCore.Door): void;
  currentMode?: number;
  door: RomiCore.Door;
  doorState?: RomiCore.DoorState;
}

export interface DoorProps extends Omit<DoorContainerProps, 'onClick'> {
  v1: number[];
  v2: number[];
  onClick?(e: React.MouseEvent<SVGGElement>): void;
}

export const getDoorStyle = (classes: any, currentMode: number | undefined) => {
  switch (currentMode) {
    case DoorMode.OPEN:
      return classes.doorOpen;
    case DoorMode.PROCESS:
      return classes.doorProcess;
    default:
      return classes.doorClose;
  }
};

const Door = React.forwardRef(function(props: DoorContainerProps): React.ReactElement {
  const { door, onClick, currentMode } = props;
  const { door_type: doorType } = door;
  const v1 = [door.v1_x, door.v1_y];
  const v2 = [door.v2_x, door.v2_y];

  const handleClick = (event: React.MouseEvent<SVGGElement, MouseEvent>) => {
    onClick && onClick(event, door);
  };

  return (
    <>
      {doorType === DoorType.SINGLE_SWING && (
        <SingleHingeDoor
          v1={v1}
          v2={v2}
          door={door}
          onClick={handleClick}
          currentMode={currentMode}
        />
      )}
      {doorType === DoorType.DOUBLE_SWING && (
        <DoubleHingeDoor
          v1={v1}
          v2={v2}
          door={door}
          onClick={handleClick}
          currentMode={currentMode}
        />
      )}
      {(doorType === DoorType.SINGLE_SLIDING || doorType === DoorType.SINGLE_TELESCOPE) && (
        <SingleSlideDoor
          v1={v1}
          v2={v2}
          door={door}
          onClick={handleClick}
          currentMode={currentMode}
        />
      )}
      {(doorType === DoorType.DOUBLE_SLIDING || doorType === DoorType.DOUBLE_TELESCOPE) && (
        <DoubleSlideDoor
          v1={v1}
          v2={v2}
          door={door}
          onClick={handleClick}
          currentMode={currentMode}
        />
      )}
      {doorType === DoorType.UNDEFINED && <DefaultDoor v1={v1} v2={v2} onClick={handleClick} />}
      {currentMode === DoorMode.PROCESS && (
        <g>
          <Spinner cx={door.v1_x} cy={-door.v1_y} r={0.4} strokeWidth={0.1}></Spinner>
        </g>
      )}
    </>
  );
});

export default Door;
