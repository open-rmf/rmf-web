import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import { RomiCoreLift } from './lift-overlay';
import Door from './door';
import { UpArrow, DownArrow } from './arrow';

const MOTION_STOPPED = 0;
const MOTION_UP = 1;
const MOTION_DOWN = 2;
const MOTION_UNKNOWN = 3;

const MODE_UNKNOWN = 0;
const MODE_HUMAN = 1;
const MODE_AGV = 2;
const MODE_FIRE = 3;
const MODE_OFFLINE = 4;
const MODE_EMERGENCY = 5;

export interface LiftProps {
  currentFloor: string;
  // TODO: this should be replaced by RomiCore.Lift once we addressed this
  // https://github.com/osrf/romi-js-core-interfaces/issues/4
  lift: RomiCoreLift;
  liftState?: RomiCore.LiftState;
  onClick?(e: React.MouseEvent<SVGGElement>, lift: RomiCoreLift): void;
}

const getLiftStyle = (
  classes: any,
  currentMode: number | undefined,
  doorState: number | undefined,
  motionState: number | undefined,
  isInCurrentFloor: boolean,
) => {
  const isDangerous = currentMode === MODE_EMERGENCY || currentMode === MODE_FIRE;
  const isOffLine = currentMode === MODE_OFFLINE || currentMode === MODE_UNKNOWN;
  if (isDangerous) return classes.danger;
  if (isOffLine) return classes.offLine;
  return isInCurrentFloor ? classes.liftOnCurrentFloor : classes.liftOnAnotherFloor;
};

const calculateMiddleCoordsOfRect = (
  x: number,
  y: number,
  width: number | undefined,
  height: number | undefined,
) => {
  // TODO: the width and height should be not undefined we addressed this
  // https://github.com/osrf/romi-js-core-interfaces/issues/4
  if (!width || !height) throw new Error('Width and Height cannot be undefined');
  return { x: x - width / 2, y: y + height / 2 };
};

const Lift = React.forwardRef(function(
  props: LiftProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const { lift, onClick, liftState, currentFloor } = props;
  const { width, depth, ref_x: x, ref_y: y, ref_yaw, doors } = lift;
  // The svg start drawing from the top left coordinate, and the backend sends us the middle X,Y so we need to transform that.
  const { x: topVerticeX, y: topVerticey } = calculateMiddleCoordsOfRect(x, y, width, depth);

  // Get properties from lift state
  const isInCurrentFloor = liftState?.current_floor === currentFloor;
  const currentMode = liftState?.current_mode;
  const doorState = liftState?.door_state;
  const motionState = liftState?.motion_state;

  const classes = useStyles();
  const liftStyle = getLiftStyle(classes, currentMode, doorState, motionState, isInCurrentFloor);
  return (
    <>
      <g ref={ref} onClick={e => onClick && onClick(e as any, lift)}>
        <rect
          className={`${classes.liftMarker} ${classes.lift} ${liftStyle}`}
          width={width}
          height={depth}
          x={topVerticeX}
          y={-topVerticey}
          transform={`rotate(${ref_yaw},${x},${-y})`}
        />
        {motionState === MOTION_STOPPED && (
          <text className={classes.liftText} x={x} y={-y}>
            Stopped
          </text>
        )}
        {motionState === MOTION_UNKNOWN && (
          <text className={classes.liftText} x={x} y={-y}>
            ?
          </text>
        )}
      </g>
      {doors.map(door => (
        <Door key={`lift-door-${door.name}`} door={door} currentMode={doorState} />
      ))}
      {motionState === MOTION_UP && <UpArrow x={x} y={-y} size={0.04} />}
      {motionState === MOTION_DOWN && <DownArrow x={x} y={-y} size={0.04} />}
    </>
  );
});

export default Lift;

const useStyles = makeStyles(() => ({
  liftMarker: {
    cursor: 'pointer',
    pointerEvents: 'auto',
  },
  lift: {
    strokeWidth: '0.2',
  },
  liftOnCurrentFloor: {
    fill: 'green',
    opacity: '50%',
  },
  liftOnAnotherFloor: {
    fill: 'grey',
  },
  danger: {
    stroke: 'red',
    fill: 'red',
  },
  offLine: {
    stroke: 'yellow',
    fill: 'yellow',
  },
  liftText: {
    dominantBaseline: 'central',
    textAnchor: 'middle',
    fontSize: '0.3px',
  },
}));
