import { styled } from '@material-ui/core';
import clsx from 'clsx';
import Debug from 'debug';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { fromRmfCoords, fromRmfYaw, radiansToDegrees } from '../geometry-utils';
import { DoorMarker } from './door-marker';

const debug = Debug('Map:LiftMarker');

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

function toDoorMode(liftState: RmfModels.LiftState): RmfModels.DoorMode {
  // LiftState uses its own enum definition of door state/mode which is separated from DoorMode.
  // But their definitions are equal so we can skip conversion.
  return { value: liftState.door_state };
}

export interface LiftMarkerProps extends React.PropsWithRef<React.SVGProps<SVGGElement>> {
  lift: RmfModels.Lift;
  liftState?: RmfModels.LiftState;
  /**
   * Whether the component should perform a translate transform to put it inline with the position
   * in RMF.
   *
   * default: true
   */
  translate?: boolean;
  variant?: keyof typeof liftMarkerClasses;
}

export const LiftMarker = React.forwardRef(function (
  { lift, liftState, variant, translate = true, ...otherProps }: LiftMarkerProps,
  ref: React.Ref<SVGGElement>,
): JSX.Element {
  debug(`render ${lift.name}`);

  const { width, depth, ref_x, ref_y, ref_yaw, doors } = lift;
  const pos = fromRmfCoords([ref_x, ref_y]);
  // Get properties from lift state
  const doorMode = liftState ? toDoorMode(liftState) : undefined;
  const markerClass = variant ? liftMarkerClasses[variant] : liftMarkerClasses.onCurrentLevel;

  /**
   * In order to keep consistent spacing, we render at a "unit box" scale it according to the
   * dimensionals of the lift.
   */
  const renderStatusText = () => {
    // QN: do we need to take into account rotation?
    const textScale = Math.min(width, depth); // keep aspect ratio
    return liftState ? (
      <text className={liftMarkerClasses.text} transform={`scale(${textScale})`}>
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
      <text className={liftMarkerClasses.text} transform={`scale(${textScale})`}>
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
      {/* it is easier to render it translate, and reverse the translation here */}
      <g transform={!translate ? `translate(${-pos[0]} ${-pos[1]})` : undefined}>
        <g transform={`translate(${pos[0]} ${pos[1]})`}>
          <rect
            className={`${liftMarkerClasses.lift} ${markerClass}`}
            width={width}
            height={depth}
            x={-width / 2}
            y={-depth / 2}
            rx="0.1"
            ry="0.1"
            transform={`rotate(${radiansToDegrees(fromRmfYaw(ref_yaw))})`}
          />
          {renderStatusText()}
        </g>
        <g>
          {doors.map((door, i) => (
            <DoorMarker key={i} door={door} doorMode={doorMode?.value} translate={true} />
          ))}
        </g>
      </g>
    </LiftMarkerRoot>
  );
});

export default LiftMarker;
