/**
 * TODO: Need door location, (v1_x, v1_y) only defines the location of the hinge, we also need the
 * length and orientation of the door to draw it on the map.
 */

import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React, { createContext, useContext } from 'react';
import { viewBoxFromLeafletBounds } from '../../util/css-utils';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';

import Door from './door';
export interface DoorsOverlayProps extends SVGOverlayProps {
  doors: readonly RomiCore.Door[];
  onDoorClick?(door: RomiCore.Door): void;
}

export default function DoorsOverlay(props: DoorsOverlayProps): React.ReactElement {
  const { doors, onDoorClick, ...otherProps } = props;
  const viewBox = viewBoxFromLeafletBounds(props.bounds);
  const useStateValue = () => useContext(DoorStateContext);
  const doorsState = useStateValue();
  return (
    <>
      <SVGOverlay {...otherProps}>
        <svg viewBox={viewBox}>
          {doors.map(door => (
            <Door
              key={`img-${door.name}`}
              door={door}
              onClick={(_, door) => onDoorClick && onDoorClick(door)}
              doorState={doorsState && doorsState[door.name]}
            />
          ))}
        </svg>
      </SVGOverlay>
    </>
  );
}

export const DoorStateContext = createContext<Readonly<Record<string, RomiCore.DoorState>>>({});
