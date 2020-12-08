import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React, { useContext } from 'react';
import { LiftMarker as LiftMarker_, LiftMarkerProps, useLiftMarkerStyles } from 'react-components';
import { viewBoxFromLeafletBounds } from '../../util/css-utils';
import { LiftStateContext } from '../rmf-contexts';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';

const debug = Debug('ScheduleVisualizer:LiftsOverlay');
const LiftMarker = React.memo(LiftMarker_);

const getLiftModeVariant = (
  currentFloor: string,
  liftState?: RomiCore.LiftState,
): keyof ReturnType<typeof useLiftMarkerStyles> | undefined => {
  if (!liftState) return 'unknown';

  const liftMode = liftState.current_mode;

  if (liftMode === RomiCore.LiftState.MODE_FIRE) return 'fire';
  if (liftMode === RomiCore.LiftState.MODE_EMERGENCY) return 'emergency';
  if (liftMode === RomiCore.LiftState.MODE_OFFLINE) return 'offLine';
  if (liftState.current_floor === currentFloor) {
    if (liftMode === RomiCore.LiftState.MODE_HUMAN) return 'human';
    if (liftMode === RomiCore.LiftState.MODE_AGV) return 'onCurrentFloor';
  } else {
    if (liftMode === RomiCore.LiftState.MODE_HUMAN) return 'moving';
    if (liftMode === RomiCore.LiftState.MODE_AGV) return 'moving';
  }
  if (liftMode === RomiCore.LiftState.MODE_UNKNOWN) return 'unknown';

  return 'unknown';
};

export interface LiftsOverlayProps extends SVGOverlayProps {
  currentFloor: string;
  lifts: readonly RomiCore.Lift[];
  onLiftClick?(lift: RomiCore.Lift): void;
}

export const LiftsOverlay = (props: LiftsOverlayProps) => {
  debug('render');

  const { lifts, onLiftClick, currentFloor, ...otherProps } = props;
  const viewBox = viewBoxFromLeafletBounds(props.bounds);
  const liftsState = useContext(LiftStateContext);

  const handleLiftClick = React.useCallback<Required<LiftMarkerProps>['onClick']>(
    (_, lift) => onLiftClick && onLiftClick(lift),
    [onLiftClick],
  );

  return (
    <>
      <SVGOverlay {...otherProps}>
        <svg viewBox={viewBox}>
          {lifts.map((lift) => (
            <LiftMarker
              key={lift.name}
              id={`Lift-${lift.name}`}
              lift={lift}
              onClick={handleLiftClick}
              liftState={liftsState && liftsState[lift.name]}
              variant={liftsState && getLiftModeVariant(currentFloor, liftsState[lift.name])}
            />
          ))}
        </svg>
      </SVGOverlay>
    </>
  );
};

export default LiftsOverlay;
