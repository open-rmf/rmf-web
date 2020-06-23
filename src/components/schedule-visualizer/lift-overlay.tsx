import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React, { createContext, useContext } from 'react';
import { viewBoxFromLeafletBounds } from '../../util/css-utils';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';
import Lift from './lift';

export interface LiftsOverlayProps extends SVGOverlayProps {
  currentFloor: string;
  lifts: readonly RomiCore.Lift[];
  onLiftClick?(lift: RomiCore.Lift): void;
}

export default function LiftsOverlay(props: LiftsOverlayProps): React.ReactElement {
  const { lifts, onLiftClick, currentFloor, ...otherProps } = props;
  const viewBox = viewBoxFromLeafletBounds(props.bounds);
  const liftsState = useContext(LiftStateContext);
  return (
    <>
      <SVGOverlay {...otherProps}>
        <svg viewBox={viewBox}>
          {lifts.map(lift => (
            <Lift
              key={`lift-${lift.name}`}
              id={`Lift-${lift.name}`}
              lift={lift}
              onClick={(_, lift) => onLiftClick && onLiftClick(lift)}
              liftState={liftsState && liftsState[lift.name]}
              currentFloor={currentFloor}
            />
          ))}
        </svg>
      </SVGOverlay>
    </>
  );
}

export const LiftStateContext = createContext<Readonly<Record<string, RomiCore.LiftState>>>({});
