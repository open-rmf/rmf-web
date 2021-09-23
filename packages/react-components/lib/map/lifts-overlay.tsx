import React from 'react';
import * as RmfModels from 'rmf-models';
import { fromRmfCoords } from '../utils';
import { useAutoScale } from './hooks';
import { ScaledNameLabel } from './label-marker';
import { LiftMarker as LiftMarker_, LiftMarkerProps, useLiftMarkerStyles } from './lift-marker';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';
import { viewBoxFromLeafletBounds } from './utils';

interface BoundedMarkerProps extends Omit<LiftMarkerProps, 'onClick'> {
  onClick?: (ev: React.MouseEvent, lift: string) => void;
}

/**
 * Bind a marker to include the lift name in the click event.
 * This is needed to avoid re-rendering all markers when only one of them changes.
 */
function bindMarker(MarkerComponent: React.ComponentType<LiftMarkerProps>) {
  return ({ onClick, ...otherProps }: BoundedMarkerProps) => {
    const handleClick = React.useCallback((ev) => onClick && onClick(ev, otherProps.lift.name), [
      onClick,
      otherProps.lift.name,
    ]);
    return <MarkerComponent onClick={onClick && handleClick} {...otherProps} />;
  };
}

const LiftMarker = React.memo(bindMarker(LiftMarker_));

export const getLiftModeVariant = (
  currentLevel: string,
  liftStateMode?: number,
  liftStateFloor?: string,
): keyof ReturnType<typeof useLiftMarkerStyles> | undefined => {
  if (!liftStateMode && !liftStateFloor) return 'unknown';
  if (liftStateMode === RmfModels.LiftState.MODE_FIRE) return 'fire';
  if (liftStateMode === RmfModels.LiftState.MODE_EMERGENCY) return 'emergency';
  if (liftStateMode === RmfModels.LiftState.MODE_OFFLINE) return 'offLine';
  if (liftStateFloor === currentLevel) {
    if (liftStateMode === RmfModels.LiftState.MODE_HUMAN) return 'human';
    if (liftStateMode === RmfModels.LiftState.MODE_AGV) return 'onCurrentLevel';
  } else {
    if (liftStateMode === RmfModels.LiftState.MODE_HUMAN) return 'moving';
    if (liftStateMode === RmfModels.LiftState.MODE_AGV) return 'moving';
  }
  if (liftStateMode === RmfModels.LiftState.MODE_UNKNOWN) return 'unknown';

  return 'unknown';
};

export interface LiftsOverlayProps extends SVGOverlayProps {
  currentLevel: string;
  lifts: RmfModels.Lift[];
  liftStates?: Record<string, RmfModels.LiftState>;
  onLiftClick?: (ev: React.MouseEvent, lift: string) => void;
}

export const LiftsOverlay = ({
  lifts,
  liftStates = {},
  onLiftClick,
  currentLevel,
  bounds,
  ...otherProps
}: LiftsOverlayProps): JSX.Element => {
  const viewBox = viewBoxFromLeafletBounds(bounds);
  const scale = useAutoScale(40);

  return (
    <SVGOverlay bounds={bounds} {...otherProps}>
      <svg viewBox={viewBox}>
        {lifts.map((lift) => {
          const pos = fromRmfCoords([lift.ref_x, lift.ref_y]);
          return (
            <g key={lift.name}>
              <LiftMarker
                lift={lift}
                onClick={onLiftClick}
                liftState={liftStates && liftStates[lift.name]}
                variant={getLiftModeVariant(
                  currentLevel,
                  liftStates[lift.name]?.current_mode,
                  liftStates[lift.name]?.current_floor,
                )}
                translate
                transform={`scale(${scale})`}
                transform-origin={`${pos[0]} ${pos[1]}`}
                aria-label={lift.name}
              />
              <ScaledNameLabel
                sourceX={pos[0]}
                sourceY={pos[1]}
                sourceRadius={Math.min(lift.width / 2, lift.depth / 2)}
                arrowLength={Math.max((lift.width / 3) * scale, (lift.depth / 3) * scale)}
                text={lift.name}
              />
            </g>
          );
        })}
      </svg>
    </SVGOverlay>
  );
};

export default LiftsOverlay;
