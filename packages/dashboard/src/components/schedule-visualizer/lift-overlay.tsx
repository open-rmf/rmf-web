import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React, { useContext } from 'react';
import { viewBoxFromLeafletBounds } from '../../util/css-utils';
import { LiftStateContext } from '../rmf-contexts';
import Lift, { LiftProps } from './lift';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';

const debug = Debug('ScheduleVisualizer:LiftsOverlay');

export interface LiftsOverlayProps extends SVGOverlayProps {
  currentFloor: string;
  lifts: readonly RomiCore.Lift[];
  onLiftClick?(lift: RomiCore.Lift): void;
}

export const LiftsOverlay = React.memo((props: LiftsOverlayProps) => {
  debug('render');

  const { lifts, onLiftClick, currentFloor, ...otherProps } = props;
  const viewBox = viewBoxFromLeafletBounds(props.bounds);
  const liftsState = useContext(LiftStateContext);

  const handleLiftClick = React.useCallback<Required<LiftProps>['onClick']>(
    (_, lift) => onLiftClick && onLiftClick(lift),
    [onLiftClick],
  );

  return (
    <>
      <SVGOverlay {...otherProps}>
        <svg viewBox={viewBox}>
          {lifts.map(lift => (
            <Lift
              key={lift.name}
              id={`Lift-${lift.name}`}
              lift={lift}
              onClick={handleLiftClick}
              liftState={liftsState && liftsState[lift.name]}
              currentFloor={currentFloor}
            />
          ))}
        </svg>
      </SVGOverlay>
    </>
  );
});

export default LiftsOverlay;
