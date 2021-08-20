import React from 'react';
import * as RmfModels from 'rmf-models';
import { LiftMarker as LiftMarker_, LiftMarkerProps, useLiftMarkerStyles } from './lift-marker';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';
import { viewBoxFromLeafletBounds } from './utils';

const LiftMarker = React.memo(LiftMarker_);

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
  onLiftClick?(lift: RmfModels.Lift): void;
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

  const handleLiftClick = React.useCallback<Required<LiftMarkerProps>['onClick']>(
    (_, lift) => onLiftClick && onLiftClick(lift),
    [onLiftClick],
  );

  return (
    <SVGOverlay bounds={bounds} {...otherProps}>
      <svg viewBox={viewBox}>
        {lifts.map((lift) => (
          <LiftMarker
            key={lift.name}
            id={`Lift-${lift.name}`}
            lift={lift}
            onClick={handleLiftClick}
            liftState={liftStates && liftStates[lift.name]}
            variant={getLiftModeVariant(
              currentLevel,
              liftStates[lift.name]?.current_mode,
              liftStates[lift.name]?.current_floor,
            )}
          />
        ))}
      </svg>
    </SVGOverlay>
  );
};

export default LiftsOverlay;
