import { styled } from '@mui/material';
import clsx from 'clsx';
import React from 'react';
import * as RmfModels from 'rmf-models';

// Gets the text to insert to the lift, the text depend on the current mode, motion state and the
// current and destination floor of the lift.
function getLiftModeText(liftState: RmfModels.LiftState): string {
  if (!liftState.current_mode) {
    return 'UNKNOWN';
  }
  switch (liftState.current_mode) {
    case RmfModels.LiftState.MODE_FIRE:
      return 'FIRE!';
    case RmfModels.LiftState.MODE_EMERGENCY:
      return 'EMERGENCY!';
    case RmfModels.LiftState.MODE_OFFLINE:
      return 'OFFLINE';
    default:
      return 'NORMAL';
  }
}

function getLiftMotionText(liftState: RmfModels.LiftState): string {
  switch (liftState.motion_state) {
    case RmfModels.LiftState.MOTION_UP:
      return '▲';
    case RmfModels.LiftState.MOTION_DOWN:
      return '▼';
    case RmfModels.LiftState.MOTION_STOPPED:
      return '⯀';
    default:
      return '?';
  }
}

export const liftMarkerClasses = {
  marker: 'lift-marker-root',
  lift: 'lift-marker-lift',
  text: 'lift-marker-text',
  onCurrentLevel: 'lift-marker-oncurrentlevel',
  moving: 'lift-marker-moving',
  unknown: 'lift-marker-unknown',
  emergency: 'lift-marker-emergency',
  fire: 'lift-marker-fire',
  offLine: 'lift-marker-offline',
  human: 'lift-marker-human',
};

const LiftMarkerRoot = styled('g')(() => ({
  [`&.${liftMarkerClasses.marker}`]: {
    cursor: 'pointer',
    pointerEvents: 'auto',
  },
  [`& .${liftMarkerClasses.lift}`]: {
    strokeWidth: '0.2',
  },
  [`& .${liftMarkerClasses.text}`]: {
    dominantBaseline: 'central',
    textAnchor: 'middle',
    fontSize: '0.16px',
    fontWeight: 'bold',
    cursor: 'inherit',
    userSelect: 'none',
  },
  [`& .${liftMarkerClasses.onCurrentLevel}`]: {
    fill: 'green',
    opacity: '70%',
  },
  [`& .${liftMarkerClasses.moving}`]: {
    fill: 'grey',
    opacity: '70%',
  },
  [`& .${liftMarkerClasses.unknown}`]: {
    fill: '#3d3c3c',
    opacity: '80%',
  },
  [`& .${liftMarkerClasses.emergency}`]: {
    fill: 'red',
    opacity: '80%',
  },
  [`& .${liftMarkerClasses.fire}`]: {
    fill: '#ff562a',
    opacity: '80%',
  },
  [`& .${liftMarkerClasses.offLine}`]: {
    fill: 'yellow',
    opacity: '80%',
  },
  [`& .${liftMarkerClasses.human}`]: {
    fill: '#90dfef',
    opacity: '80%',
  },
}));

export interface LiftMarkerProps extends React.PropsWithRef<React.SVGProps<SVGGElement>> {
  cx: number;
  cy: number;
  width: number;
  height: number;
  yaw: number;
  liftState?: RmfModels.LiftState;
  variant?: keyof typeof liftMarkerClasses;
}

export const LiftMarker = React.forwardRef(function (
  { cx, cy, width, height, yaw, liftState, variant, ...otherProps }: LiftMarkerProps,
  ref: React.Ref<SVGGElement>,
): JSX.Element {
  const markerClass = variant ? liftMarkerClasses[variant] : liftMarkerClasses.onCurrentLevel;
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
      <text
        className={liftMarkerClasses.text}
        transform={`translate(${cx} ${cy}) scale(${textScale})`}
      >
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
      <text
        className={liftMarkerClasses.text}
        transform={`translate(${cx} ${cy}) scale(${textScale})`}
      >
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
    <LiftMarkerRoot
      ref={ref}
      className={clsx(
        otherProps.onClick ? liftMarkerClasses.marker : undefined,
        otherProps.className,
      )}
      {...otherProps}
    >
      <rect
        className={`${liftMarkerClasses.lift} ${markerClass}`}
        x={x}
        y={y}
        width={width}
        height={height}
        rx={r}
        ry={r}
        style={{ transform: `rotate(${yaw}deg)`, transformOrigin: `${cx}px ${cy}px` }}
      />
      {renderStatusText()}
    </LiftMarkerRoot>
  );
});

export default LiftMarker;
