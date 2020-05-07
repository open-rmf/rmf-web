import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React, { createContext, useContext } from 'react';
import { viewBoxFromLeafletBounds } from '../../util/css-utils';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';
import Lift from './lift';

export interface LiftsOverlayProps extends SVGOverlayProps {
  currentLevel: string;
  lifts: readonly RomiCore.Lift[];
  onLiftClick?(lift: RomiCore.Lift): void;
}

export default function LiftsOverlay(props: LiftsOverlayProps): React.ReactElement {
  const { lifts, onLiftClick, currentLevel, ...otherProps } = props;
  const viewBox = viewBoxFromLeafletBounds(props.bounds);
  const useStateValue = () => useContext(LiftStateContext);
  const liftsState = useStateValue();
  console.log(lifts);
  return (
    <>
      <SVGOverlay {...otherProps}>
        <svg viewBox={viewBox}>
          {lifts.map(lift => (
            <Lift
              key={`lift-${lift.name}`}
              lift={lift}
              onClick={(_, lift) => onLiftClick && onLiftClick(lift)}
              liftState={liftsState && liftsState[lift.name]}
              currentLevel={currentLevel}
            />
          ))}
        </svg>
      </SVGOverlay>
    </>
  );
}

export const LiftStateContext = createContext<Readonly<Record<string, RomiCore.LiftState>>>({});
