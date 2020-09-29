import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import DefaultDoor from './door-default';
import DoubleHingeDoor from './door-double-hinge';
import DoubleSlideDoor from './door-double-slide';
import SingleHingeDoor from './door-single-hinge';
import SingleSlideDoor from './door-single-slide';

const debug = Debug('ScheduleVisualizer:Door');

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

export const getDoorStyle = (classes: Record<string, string>, currentMode: number | undefined) => {
  const { MODE_OPEN, MODE_MOVING } = RomiCore.DoorMode;

  switch (currentMode) {
    case MODE_OPEN:
      return classes.doorOpen;
    case MODE_MOVING:
      return classes.doorProcess;
    default:
      return classes.doorClose;
  }
};

const Door = React.memo(
  React.forwardRef(function(
    props: DoorContainerProps,
    ref: React.Ref<SVGGElement>,
  ): React.ReactElement {
    const { door, onClick, currentMode } = props;
    debug('render %s', door.name);

    const {
      DOOR_TYPE_UNDEFINED,
      DOOR_TYPE_SINGLE_SLIDING,
      DOOR_TYPE_DOUBLE_SLIDING,
      DOOR_TYPE_SINGLE_TELESCOPE,
      DOOR_TYPE_DOUBLE_TELESCOPE,
      DOOR_TYPE_SINGLE_SWING,
      DOOR_TYPE_DOUBLE_SWING,
    } = RomiCore.Door;
    const { door_type: doorType } = door;
    const v1 = [door.v1_x, door.v1_y];
    const v2 = [door.v2_x, door.v2_y];

    const handleClick = (event: React.MouseEvent<SVGGElement, MouseEvent>) => {
      onClick && onClick(event, door);
    };

    return (
      <g ref={ref} data-component="Door" aria-label={door.name}>
        {doorType === DOOR_TYPE_SINGLE_SWING && (
          <SingleHingeDoor
            v1={v1}
            v2={v2}
            door={door}
            onClick={handleClick}
            currentMode={currentMode}
          />
        )}
        {doorType === DOOR_TYPE_DOUBLE_SWING && (
          <DoubleHingeDoor
            v1={v1}
            v2={v2}
            door={door}
            onClick={handleClick}
            currentMode={currentMode}
          />
        )}
        {(doorType === DOOR_TYPE_SINGLE_SLIDING || doorType === DOOR_TYPE_SINGLE_TELESCOPE) && (
          <SingleSlideDoor
            v1={v1}
            v2={v2}
            door={door}
            onClick={handleClick}
            currentMode={currentMode}
          />
        )}
        {(doorType === DOOR_TYPE_DOUBLE_SLIDING || doorType === DOOR_TYPE_DOUBLE_TELESCOPE) && (
          <DoubleSlideDoor
            v1={v1}
            v2={v2}
            door={door}
            onClick={handleClick}
            currentMode={currentMode}
          />
        )}
        {doorType === DOOR_TYPE_UNDEFINED && <DefaultDoor v1={v1} v2={v2} onClick={handleClick} />}
      </g>
    );
  }),
);

export default Door;
