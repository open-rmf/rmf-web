import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React, { useContext } from 'react';
import { DoorStateContext } from '../app';
import { Spinner } from './spinner';

const OPEN = 0;
const PROCESS = 1;
const CLOSE = 2;

// Door Type
const DOOR_TYPE_SINGLE_SLIDING = 1;
const DOOR_TYPE_DOUBLE_SLIDING = 2;
export interface DoorProps {
  door: RomiCore.Door;
  onClick?(e: React.MouseEvent<SVGGElement>, place: RomiCore.Door): void;
}

class DoorManager {
  static getNewCoords(v1: number[], v2: number[], angle: number, direction: number, process = 0) {
    const [x1, y1] = v1;
    const distance = this.getDistance(v1, v2);
    const initialAngle = this.getAngleBetweenTwoPointsDegrees(v1, v2);
    const coordX = x1 + distance * Math.cos(initialAngle + angle * direction * process);
    const coordY = y1 + distance * Math.sin(initialAngle + angle * direction * process);
    return [coordX, coordY];
  }

  static getDistance(v1: number[], v2: number[]) {
    const [x1, y1] = v1;
    const [x2, y2] = v2;
    return Math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2);
  }

  static getAngleBetweenTwoPointsDegrees(v1: number[], v2: number[]) {
    const [x1, y1] = v1;
    const [x2, y2] = v2;
    return (Math.atan2(y2 - y1, x2 - x1) * 180) / Math.PI;
  }

  static getAngleBetweenTwoPointsRadians(v1: number[], v2: number[]) {
    const [x1, y1] = v1;
    const [x2, y2] = v2;
    return Math.atan2(y2 - y1, x2 - x1);
  }
}

const SingleHingeDoor = React.forwardRef(function(
  props: DoorProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const { door, onClick } = props;
  const { motion_range, motion_direction, v1_x, v1_y, v2_x, v2_y } = door;
  const [hingeX, hingeY, extendX, extendY] = [v1_x, v1_y, v2_x, v2_y];

  const classes = useStyles();
  const useStateValue = () => useContext(DoorStateContext);

  console.log(props);
  console.log(useStateValue());

  // Door state
  const doorState = useStateValue() as any;
  const currentDoor = doorState && doorState[door.name];
  let currentMode = OPEN;
  let fillColor = 'brown';
  let doorTime = 0;
  let newCoordX: number | undefined, newCoordY: number | undefined;

  if (currentDoor) {
    doorTime = currentDoor.door_time;
    currentMode = currentDoor.current_mode.value;
    fillColor = currentMode === OPEN ? 'brown' : currentMode === PROCESS ? 'blue' : 'red';
    [newCoordX, newCoordY] = DoorManager.getNewCoords(
      [hingeX, hingeY],
      [extendX, extendY],
      motion_range,
      motion_direction,
      doorTime,
    );
  }

  return (
    <>
      <g ref={ref} onClick={e => onClick && onClick(e as any, door)}>
        <line
          className={classes.doorMarker}
          x1={hingeX}
          y1={-hingeY}
          x2={newCoordX || extendX}
          y2={(newCoordY || extendY) * -1}
          stroke={fillColor}
          stroke-width="0.2"
        />
      </g>
      {currentMode === PROCESS && (
        <g>
          <Spinner cx={hingeX} cy={-hingeY} r={0.4} strokeWidth={0.1}></Spinner>
        </g>
      )}
    </>
  );
});

const Door = React.forwardRef(function(
  props: DoorProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const { door, onClick } = props;
  const { door_type: doorType } = door;
  // Door extends till

  // const x = Math.min(door.v1_x, door.v2_x);
  // const y = -1 * Math.min(door.v1_y, door.v2_y);
  // const maxX = Math.max(door.v2_x, door.v1_x);
  // const maxY = Math.max(door.v2_y, door.v1_y);
  // const width = Math.abs(door.v2_x - door.v1_x);
  // const height = Math.abs(door.v2_y - door.v1_y);

  return (
    <>
      {doorType === DOOR_TYPE_SINGLE_SLIDING && (
        <SingleHingeDoor door={door} onClick={onClick}></SingleHingeDoor>
      )}
      {doorType === DOOR_TYPE_DOUBLE_SLIDING && (
        <SingleHingeDoor door={door} onClick={onClick}></SingleHingeDoor>
      )}
    </>
  );
});

export default Door;

const useStyles = makeStyles(() => ({
  doorMarker: {
    cursor: 'pointer',
    pointerEvents: 'auto',
  },
}));
