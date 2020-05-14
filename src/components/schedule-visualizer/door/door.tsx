import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import { Spinner } from '../spinner';
import doorStyles from './door-style';
import SingleHingeDoor from './door-single-hinge';
import DoubleHingeDoor from './door-double-hinge';

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
 * currentMode: current mode of the door. E.g: 0 = CLOSE
 * door: door information on map
 * doorState: current state of the door.
 * onClick: action to trigger on click.
 */
export interface DoorProps {
  currentMode?: number;
  door: RomiCore.Door;
  doorState?: RomiCore.DoorState;
  onClick?(e: React.MouseEvent<SVGGElement>, door: RomiCore.Door): void;
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

const Door = React.forwardRef(function(
  props: DoorProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const { door, onClick, currentMode } = props;
  const { door_type: doorType } = door;
  return (
    <>
      {doorType === DoorType.SINGLE_SLIDING && (
        <SingleHingeDoor door={door} onClick={onClick} currentMode={currentMode} />
      )}
      {doorType === DoorType.DOUBLE_SLIDING && (
        <DoubleHingeDoor door={door} onClick={onClick} currentMode={currentMode} />
      )}
      {currentMode === DoorMode.PROCESS && (
        <g>
          <Spinner cx={door.v1_x} cy={-door.v1_y} r={0.4} strokeWidth={0.1}></Spinner>
        </g>
      )}
    </>
  );
});

export default Door;
