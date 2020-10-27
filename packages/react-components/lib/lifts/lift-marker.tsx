import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import { DoorMarker } from '..';
import { joinClasses } from '../css-utils';
import { radiansToDegrees, transformMiddleCoordsOfRectToSVGBeginPoint } from '../geometry-utils';

const debug = Debug('Lifts:LiftMarker');

// Gets the text to insert to the lift, the text depend on the current mode, motion state and the
// current and destination floor of the lift.
function getLiftModeText(liftState: RomiCore.LiftState): string {
  if (!liftState.current_mode) {
    return 'UNKNOWN';
  }
  switch (liftState.current_mode) {
    case RomiCore.LiftState.MODE_FIRE:
      return 'FIRE!';
    case RomiCore.LiftState.MODE_EMERGENCY:
      return 'EMERGENCY!';
    case RomiCore.LiftState.MODE_OFFLINE:
      return 'OFFLINE';
    default:
      return 'NORMAL';
  }
}

function getLiftMotionText(liftState: RomiCore.LiftState): string {
  switch (liftState.motion_state) {
    case RomiCore.LiftState.MOTION_UP:
      return '▲';
    case RomiCore.LiftState.MOTION_DOWN:
      return '▼';
    case RomiCore.LiftState.MOTION_STOPPED:
      return '⯀';
    default:
      return '?';
  }
}

const useStyles = makeStyles({
  marker: {
    cursor: 'pointer',
    pointerEvents: 'auto',
  },
  lift: {
    strokeWidth: '0.2',
  },
  text: {
    dominantBaseline: 'central',
    textAnchor: 'middle',
    fontSize: '0.16px',
    fontWeight: 'bold',
    cursor: 'inherit',
    userSelect: 'none',
  },
});

const useMarkerStyles = makeStyles({
  onCurrentFloor: {
    fill: 'green',
    opacity: '70%',
  },
  moving: {
    fill: 'grey',
    opacity: '70%',
  },
  unknown: {
    fill: '#3d3c3c',
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
  human: {
    fill: '#90dfef',
    opacity: '80%',
  },
});

/**
 * Gets the style to apply to the lift, the styles depend on the current mode of the lift.
 * @param currentMode current mode of the elevator
 * @param isInCurrentFloor if the lift is in the current floor return true.
 */
function useMarkerStyle(isInCurrentFloor: boolean, liftState?: RomiCore.LiftState): string {
  const classes = useMarkerStyles();
  if (!liftState) {
    return classes.unknown;
  }

  switch (liftState.current_mode) {
    case RomiCore.LiftState.MODE_EMERGENCY:
      return classes.emergency;
    case RomiCore.LiftState.MODE_FIRE:
      return classes.fire;
    case RomiCore.LiftState.MODE_OFFLINE:
      return classes.offLine;
  }

  if (!isInCurrentFloor) {
    return classes.moving;
  }

  switch (liftState.current_mode) {
    case RomiCore.LiftState.MODE_AGV:
      return classes.onCurrentFloor;
    case RomiCore.LiftState.MODE_HUMAN:
      return classes.human;
    default:
      return classes.unknown;
  }
}

function toDoorMode(liftState: RomiCore.LiftState): RomiCore.DoorMode {
  // LiftState uses its own enum definition of door state/mode which is separated from DoorMode.
  // But their definitions are equal so we can skip conversion.
  return { value: liftState.door_state };
}

export interface LiftMarkerProps {
  isInCurrentFloor: boolean;
  lift: RomiCore.Lift;
  liftState?: RomiCore.LiftState;
  onClick?(e: React.MouseEvent<SVGGElement>): void;
}

export const LiftMarker = React.memo(
  React.forwardRef(function (props: LiftMarkerProps, ref: React.Ref<SVGGElement>): JSX.Element {
    const { isInCurrentFloor, lift, liftState, onClick } = props;
    debug(`render ${lift.name}`);

    const { width, depth, ref_x, ref_y, ref_yaw, doors } = lift;
    // The svg start drawing from the top left coordinate, and the backend sends us the middle X,Y so we need to transform that.
    const [topVerticeX, topVerticeY] = transformMiddleCoordsOfRectToSVGBeginPoint(
      ref_x,
      ref_y,
      width,
      depth,
    );
    // Since we are working with a plane with positive X and Negative Y we need to change the sign of the y.
    const contextY = -ref_y;
    const contextTopVerticeY = -topVerticeY;
    // Get properties from lift state
    const doorMode = liftState ? toDoorMode(liftState) : undefined;

    const classes = useStyles();

    const liftStyle = useMarkerStyle(isInCurrentFloor, liftState);

    /**
     * In order to keep consistent spacing, we render at a "unit box" scale it according to the
     * dimensionals of the lift.
     */
    const renderStatusText = () => {
      // FIXME: take into account rotation
      const textScale = Math.min(width, depth); // keep aspect ratio
      return liftState ? (
        <text className={classes.text} x={ref_x} y={contextY} transform={`scale(${textScale})`}>
          <tspan x="0" dy="-1.8em">
            {liftState?.current_floor}
          </tspan>
          <tspan x="0" dy="1.2em" fontSize="0.7em">
            {getLiftModeText(liftState)}
          </tspan>
          <tspan x="0" dy="0.6em" fontSize="3em">
            {getLiftMotionText(liftState)}
          </tspan>
        </text>
      ) : (
        <text className={classes.text} x={ref_x} y={contextY} transform={`scale(${textScale})`}>
          <tspan x="0" dy="-0.5em">
            Unknown
          </tspan>
          <tspan x="0" dy="1em">
            State
          </tspan>
        </text>
      );
    };

    return (
      <g>
        <g
          ref={ref}
          onClick={onClick}
          className={joinClasses(onClick ? classes.marker : undefined)}
        >
          <rect
            className={`${classes.lift} ${liftStyle}`}
            width={width}
            height={depth}
            x={topVerticeX}
            y={contextTopVerticeY}
            rx="0.1"
            ry="0.1"
            transform={`rotate(${radiansToDegrees(ref_yaw)}, ${ref_x},${contextY})`}
          />
          {renderStatusText()}
        </g>
        <g>
          {doors.map((door, i) => (
            <DoorMarker key={i} door={door} doorMode={doorMode} />
          ))}
        </g>
      </g>
    );
  }),
);

export default LiftMarker;
