import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React, { createContext, useContext } from 'react';
import { viewBoxFromLeafletBounds } from '../../util/css-utils';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';
import LiftContainer from './lift/liftContainer';

export interface LiftsOverlayProps extends SVGOverlayProps {
  currentFloor: string;
  // TODO: this should be replaced by RomiCore.Lift once we addressed this
  // https://github.com/osrf/romi-js-core-interfaces/issues/4
  lifts: readonly RomiCoreLift[];
  onLiftClick?(lift: RomiCoreLift): void;
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
            <LiftContainer
              key={`lift-${lift.name}`}
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

// TODO: this should be replaced by RomiCore.Lift once we addressed this
// https://github.com/osrf/romi-js-core-interfaces/issues/4
export interface RomiCoreLift extends RomiCore.Lift {
  width?: number;
  depth?: number;
}
