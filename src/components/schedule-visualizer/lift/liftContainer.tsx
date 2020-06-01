import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import Door from '../door/door';
import Lift from './lift';
import { radiansToDegrees } from '../../../util/angle-calculation';
import { UpArrow, DownArrow } from '../arrow';

export interface LiftProps {
  currentFloor: string;
  lift: RomiCore.Lift;
  liftState?: RomiCore.LiftState;
  onClick?(e: React.MouseEvent<SVGGElement>, lift: RomiCore.Lift): void;
}

/**
 * Gets the style to apply to the lift, the styles depend on the current mode of the lift.
 * @param classes contains all possibles styles of an elevator
 * @param currentMode current mode of the elevator
 * @param isInCurrentFloor if the lift is in the current floor return true.
 */
const getLiftStyle = (classes: any, currentMode: number | undefined, isInCurrentFloor: boolean) => {
  const {
    MODE_AGV,
    MODE_EMERGENCY,
    MODE_FIRE,
    MODE_HUMAN,
    MODE_OFFLINE,
    MODE_UNKNOWN,
  } = RomiCore.LiftState;
  const isOffLine = currentMode === MODE_OFFLINE || currentMode === MODE_UNKNOWN;
  const isModeAGV = currentMode === MODE_AGV;
  const isModeHuman = currentMode === MODE_HUMAN;

  if (currentMode === MODE_EMERGENCY) return classes.emergency;
  if (currentMode === MODE_FIRE) return classes.fire;
  if (isOffLine) return classes.offLine;
  if (!isInCurrentFloor) return classes.liftOnAnotherFloor;
  if (isInCurrentFloor && isModeAGV) return classes.liftOnCurrentFloor;
  if (isInCurrentFloor && isModeHuman) return classes.humanMode;
};

// Gets the text to insert to the lift, the text depend on the current mode, motion state and the
// current and destination floor of the lift.
const getLiftModeText = (currentMode: number | undefined) => {
  const {
    MODE_AGV,
    MODE_EMERGENCY,
    MODE_FIRE,
    MODE_HUMAN,
    MODE_OFFLINE,
    MODE_UNKNOWN,
  } = RomiCore.LiftState;

  if (!currentMode) return 'UNKNOWN';
  if (currentMode === MODE_FIRE) return 'FIRE!';
  if (currentMode === MODE_EMERGENCY) return 'EMERGENCY!';
  if (currentMode === MODE_OFFLINE) return 'OFFLINE';
  if (currentMode === MODE_HUMAN) return 'HUMAN';
  if (currentMode === MODE_AGV) return 'AGV';
  if (currentMode === MODE_UNKNOWN) return 'UNKNOWN';
};

const getLiftMotionText = (
  currentFloor: string | undefined,
  destinationFloor: string | undefined,
  motionState: number | undefined,
) => {
  const { MOTION_DOWN, MOTION_UP, MOTION_STOPPED, MOTION_UNKNOWN } = RomiCore.LiftState;
  if (motionState === MOTION_UNKNOWN) return '?';
  if (motionState === MOTION_STOPPED) return 'STOPPED';
  if (motionState === MOTION_UP || motionState === MOTION_DOWN)
    return `${currentFloor} → ${destinationFloor}`;
};

/**
 * Transform coords on the middle of a SVG's Rect to top left coords.
 */
const transformMiddleCoordsOfRectToSVGBeginPoint = (
  x: number,
  y: number,
  width: number,
  depth: number,
) => {
  return { x: x - width / 2, y: y + depth / 2 };
};

/**
 * Get related Y coord starting from top to bottom.
 */
const getRelatedYCoord = (topY: number, height: number, percentage: number) => {
  return topY + height * percentage;
};

const LiftContainer = React.forwardRef(function(
  props: LiftProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const { lift, onClick, liftState, currentFloor } = props;
  const { width, depth, ref_x: x, ref_y: y, ref_yaw, doors } = lift;
  // The svg start drawing from the top left coordinate, and the backend sends us the middle X,Y so we need to transform that.
  const { x: topVerticeX, y: topVerticeY } = transformMiddleCoordsOfRectToSVGBeginPoint(
    x,
    y,
    width,
    depth,
  );
  // Since we are working with a plane with positive X and Negative Y we need to change the sign of the y.
  const contextY = -y;
  const contextTopVerticeY = -topVerticeY;
  // Get properties from lift state
  const currentMode = liftState?.current_mode;
  const destinationFloor = liftState?.destination_floor;
  const doorState = liftState?.door_state;
  const isInCurrentFloor = liftState?.current_floor === currentFloor;
  const motionState = liftState?.motion_state;

  const classes = liftStyles();

  const liftStyle = getLiftStyle(classes, currentMode, isInCurrentFloor);
  const liftMotionText = getLiftMotionText(liftState?.current_floor, destinationFloor, motionState);
  const liftModeText = getLiftModeText(currentMode);

  const handleLiftClick = (event: any) => {
    onClick && onClick(event, lift);
  };
  return (
    <>
      {width && depth && (
        <Lift
          ref={ref}
          lift={{
            x: topVerticeX,
            y: contextTopVerticeY,
            width: width,
            height: depth,
            rx: 0.1,
            ry: 0.1,
            transform: `rotate(${radiansToDegrees(ref_yaw)},${x},${contextY})`,
            style: liftStyle,
          }}
          liftMotionText={{
            x: x,
            y: contextY,
            text: liftMotionText,
            fontSize: '0.18px',
          }}
          liftModeText={{
            x: x,
            y: getRelatedYCoord(contextTopVerticeY, depth, 0.25),
            text: liftModeText,
            fontSize: '0.18px',
          }}
          onClick={handleLiftClick}
        />
      )}
      {motionState === RomiCore.LiftState.MOTION_UP && (
        <UpArrow x={x} y={contextY} size={0.03} padding={[0, 0.1]} />
      )}
      {motionState === RomiCore.LiftState.MOTION_DOWN && (
        <DownArrow x={x} y={contextY} size={0.03} padding={[0, 0.1]} />
      )}
      {doors.map(door => (
        <Door key={`lift-door-${door.name}`} door={door} currentMode={doorState} />
      ))}
    </>
  );
});

export default LiftContainer;

export const liftStyles = makeStyles(() => ({
  liftMarker: {
    cursor: 'pointer',
    pointerEvents: 'auto',
  },
  lift: {
    strokeWidth: '0.2',
  },
  liftOnCurrentFloor: {
    fill: 'green',
    opacity: '70%',
  },
  liftOnAnotherFloor: {
    fill: 'grey',
    opacity: '80%',
  },
  emergency: {
    fill: 'red',
    opacity: '80%',
  },
  fire: {
    fill: '#ff562a',
    opacity: '80%',
  },
  offLine: {
    fill: 'yellow',
    opacity: '80%',
  },
  humanMode: {
    fill: '#90dfef',
    opacity: '80%',
  },
}));
