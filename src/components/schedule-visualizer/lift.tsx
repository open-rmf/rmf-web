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

const getLiftStyle = (classes: any, currentMode: number | undefined, isInCurrentFloor: boolean) => {
  const isDangerous = currentMode === MODE_EMERGENCY || currentMode === MODE_FIRE;
  const isOffLine = currentMode === MODE_OFFLINE || currentMode === MODE_UNKNOWN;
  const isModeAGV = currentMode === MODE_AGV;
  const isModeHuman = currentMode === MODE_HUMAN;
  if (isDangerous) return classes.danger;
  if (isOffLine) return classes.offLine;
  return isInCurrentFloor && (isModeAGV || isModeHuman)
    ? classes.liftOnCurrentFloor
    : classes.liftOnAnotherFloor;
};

const getLiftText = (
  currentMode: number | undefined,
  motionState: number | undefined,
  destinationFloor: string | undefined,
) => {
  // if (currentMode === MODE_FIRE) return 'FIRE!';
  if (currentMode === MODE_EMERGENCY) return 'EMERGENCY!';
  if (currentMode === MODE_OFFLINE) return 'OFFLINE';

  // Motion
  if (motionState === MOTION_UNKNOWN) return '?';
  if (motionState === MOTION_STOPPED) return 'STOPPED';
  if (motionState === MOTION_UP || motionState === MOTION_UP) return `Dest: ${destinationFloor}`;
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
  const destinationFloor = liftState?.destination_floor;

  const classes = useStyles();
  const liftStyle = getLiftStyle(classes, currentMode, isInCurrentFloor);
  const liftText = getLiftText(currentMode, motionState, destinationFloor);
  return (
    <>
      <g ref={ref} onClick={e => onClick && onClick(e, lift)}>
        <rect
          className={`${classes.liftMarker} ${classes.lift} ${liftStyle}`}
          width={width}
          height={depth}
          x={topVerticeX}
          y={-topVerticey}
          transform={`rotate(${ref_yaw},${x},${-y})`}
        />
        {liftText && (
          <text className={classes.liftText} x={x} y={-y}>
            {liftText}
          </text>
        )}
      </g>
      {motionState === MOTION_UP && <UpArrow x={x} y={-y} size={0.04} />}
      {motionState === MOTION_DOWN && <DownArrow x={x} y={-y} size={0.04} />}
      {doors.map(door => (
        <Door key={`lift-door-${door.name}`} door={door} currentMode={doorState} />
      ))}
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
    opacity: '40%',
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
