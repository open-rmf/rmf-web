import { makeStyles } from '@material-ui/core';
import type { LiftState } from 'api-client';
import clsx from 'clsx';
import React from 'react';
import { LiftState as RmfLiftState } from 'rmf-models';

// Gets the text to insert to the lift, the text depend on the current mode, motion state and the
// current and destination floor of the lift.
function getLiftModeText(liftState: LiftState): string {
  if (!liftState.current_mode) {
    return 'UNKNOWN';
  }
  switch (liftState.current_mode) {
    case RmfLiftState.MODE_FIRE:
      return 'FIRE!';
    case RmfLiftState.MODE_EMERGENCY:
      return 'EMERGENCY!';
    case RmfLiftState.MODE_OFFLINE:
      return 'OFFLINE';
    default:
      return 'NORMAL';
  }
}

function getLiftMotionText(liftState: LiftState): string {
  switch (liftState.motion_state) {
    case RmfLiftState.MOTION_UP:
      return '▲';
    case RmfLiftState.MOTION_DOWN:
      return '▼';
    case RmfLiftState.MOTION_STOPPED:
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

export const useLiftMarkerStyles = makeStyles((theme) => ({
  onCurrentLevel: {
    fill: theme.palette.success.light,
    opacity: '70%',
  },
  moving: {
    fill: theme.palette.secondary.light,
    opacity: '70%',
  },
  unknown: {
    fill: theme.palette.warning.light,
    opacity: '80%',
  },
  emergency: {
    fill: theme.palette.error.light,
    opacity: '80%',
  },
  fire: {
    fill: theme.palette.error.main,
    opacity: '80%',
  },
  offLine: {
    fill: theme.palette.grey[400],
    opacity: '80%',
  },
  human: {
    fill: theme.palette.info.main,
    opacity: '80%',
  },
}));

export interface LiftMarkerProps extends React.PropsWithRef<React.SVGProps<SVGGElement>> {
  cx: number;
  cy: number;
  width: number;
  height: number;
  yaw: number;
  liftState?: LiftState;
  variant?: keyof ReturnType<typeof useLiftMarkerStyles>;
}

export const LiftMarker = React.forwardRef(function (
  { cx, cy, width, height, yaw, liftState, variant, ...otherProps }: LiftMarkerProps,
  ref: React.Ref<SVGGElement>,
): JSX.Element {
  const classes = useStyles();
  const markerClasses = useLiftMarkerStyles();
  const markerClass = variant ? markerClasses[variant] : markerClasses.onCurrentLevel;
  const x = cx - width / 2;
  const y = cy - height / 2;
  const r = Math.max(width, height) * 0.04;

  /**
   * In order to keep consistent spacing, we render at a "unit box" scale it according to the
   * dimensionals of the lift.
   */
  const renderStatusText = () => {
    // QN: do we need to take into account rotation?
    const textScale = Math.min(width, height); // keep aspect ratio
    return liftState ? (
      <text className={classes.text} transform={`translate(${cx} ${cy}) scale(${textScale})`}>
        <tspan x="0" dy="-1.8em">
          {liftState.current_floor}
        </tspan>
        <tspan x="0" dy="1.2em" fontSize="0.7em">
          {getLiftModeText(liftState)}
        </tspan>
        <tspan x="0" dy="0.6em" fontSize="3em">
          {getLiftMotionText(liftState)}
        </tspan>
      </text>
    ) : (
      <text className={classes.text} transform={`translate(${cx} ${cy}) scale(${textScale})`}>
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
    <g
      ref={ref}
      className={clsx(otherProps.onClick ? classes.marker : undefined, otherProps.className)}
      {...otherProps}
    >
      <rect
        className={`${classes.lift} ${markerClass}`}
        x={x}
        y={y}
        width={width}
        height={height}
        rx={r}
        ry={r}
        style={{ transform: `rotate(${yaw}deg)`, transformOrigin: `${cx}px ${cy}px` }}
      />
      {renderStatusText()}
    </g>
  );
});

export default LiftMarker;
