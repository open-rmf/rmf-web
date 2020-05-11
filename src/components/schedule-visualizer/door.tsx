import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import { Spinner } from './spinner';

// Door state
//eslint-disable-next-line
const CLOSE = 0;
const PROCESS = 1;
const OPEN = 2;

// Door Type
const DOOR_TYPE_SINGLE_SLIDING = 1;
const DOOR_TYPE_DOUBLE_SLIDING = 2;
export interface DoorProps {
  door: RomiCore.Door;
  onClick?(e: React.MouseEvent<SVGGElement>, door: RomiCore.Door): void;
  currentMode?: number;
  doorState?: RomiCore.DoorState;
}

const getDoorStyle = (classes: any, currentMode: number | undefined) => {
  return currentMode === OPEN
    ? classes.doorOpen
    : currentMode === PROCESS
    ? classes.doorProcess
    : classes.doorClose;
};

const SingleHingeDoor = React.forwardRef(function(
  props: DoorProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const { door, onClick, currentMode } = props;
  const { v1_x, v1_y, v2_x, v2_y } = door;
  const [hingeX, hingeY, extendX, extendY] = [v1_x, v1_y, v2_x, v2_y];
  const classes = useStyles();

  return (
    <>
      <g ref={ref} onClick={e => onClick && onClick(e as any, door)}>
        <line
          className={`${classes.doorMarker} ${classes.door} ${getDoorStyle(classes, currentMode)}`}
          x1={hingeX}
          y1={-hingeY}
          x2={extendX}
          y2={-extendY}
        />
      </g>
    </>
  );
});

/**
 * Double hinge doors:
 * - hinges are located at both (v1_x, v1_y) and (v2_x, v2_y)
 * - motion range = door swing ranges in DEGREES (assume symmetric)
 * - same motion-direction selection as single hinge
 */
const DoubleHingeDoor = React.forwardRef(function(
  props: DoorProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const { door, onClick, currentMode } = props;
  const { v1_x, v1_y, v2_x, v2_y } = door;
  const [hingeX1, hingeY1, hingeX2, hingeY2] = [v1_x, v1_y, v2_x, v2_y];
  const [extendX1, extendY1] = [hingeX1 + (v2_x - v1_x) / 2, hingeY1 + (v2_y - v1_y) / 2];
  const classes = useStyles();

  return (
    <>
      <g ref={ref} onClick={e => onClick && onClick(e as any, door)}>
        <line
          className={`${classes.doorMarker} ${classes.door} ${getDoorStyle(classes, currentMode)}`}
          x1={hingeX1}
          y1={-hingeY1}
          x2={extendX1}
          y2={-extendY1}
        />
      </g>
      <g ref={ref} onClick={e => onClick && onClick(e as any, door)}>
        <line
          className={`${classes.doorMarker} ${classes.door} ${getDoorStyle(classes, currentMode)}`}
          x1={extendX1}
          y1={-extendY1}
          x2={hingeX2}
          y2={-hingeY2}
        />
      </g>
    </>
  );
});

const Door = React.forwardRef(function(
  props: DoorProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const { door, onClick, currentMode } = props;
  const { door_type: doorType } = door;
  return (
    <>
      {doorType === DOOR_TYPE_SINGLE_SLIDING && (
        <SingleHingeDoor door={door} onClick={onClick} currentMode={currentMode} />
      )}
      {doorType === DOOR_TYPE_DOUBLE_SLIDING && (
        <DoubleHingeDoor door={door} onClick={onClick} currentMode={currentMode} />
      )}
      {currentMode === PROCESS && (
        <g>
          <Spinner cx={door.v1_x} cy={-door.v1_y} r={0.4} strokeWidth={0.1}></Spinner>
        </g>
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
  door: {
    strokeWidth: '0.2',
  },
  doorOpen: {
    stroke: '#AFDDAE',
    strokeDasharray: 0.1,
  },
  doorClose: {
    stroke: '#4A353E',
  },
  doorProcess: {
    stroke: '#E9CE9F',
    strokeDasharray: 0.3,
  },
}));
