/**
 * TODO: Need door location, (v1_x, v1_y) only defines the location of the hinge, we also need the
 * length and orientationo of the door to draw it on the map.
 */

// import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
// import * as L from 'leaflet';
import React from 'react';
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
  // const bounds =
  //   props.bounds instanceof L.LatLngBounds ? props.bounds : new L.LatLngBounds(props.bounds);

  // const width = bounds.getEast() - bounds.getWest();
  // const height = bounds.getNorth() - bounds.getSouth();
  // const viewBox = `0 0 ${100} ${100}`;
  // const fillColor = 'rgba(0, 0, 0, 0.1)';
  console.log(doors);
  return (
    <SVGOverlay {...otherProps}>
      <svg viewBox={viewBox}>
        {doors.map(door => (
          <Door
            key={`img-${door.name}`}
            door={door}
            onClick={(_, door) => onDoorClick && onDoorClick(door)}
          />
        ))}
      </svg>
    </SVGOverlay>

    // <SVGOverlay {...otherProps}>
    //   <svg viewBox={viewBox}>
    //     {doors.map(door => {
    //       console.log(door);

    //       console.log(width, height);
    //       return (
    //         <g key={door.name}>
    //           <filter id={`${door.name}-shadow`} x="-150%" y="-150%" width="400%" height="400%">
    //             <feDropShadow dx="0" dy="0" stdDeviation="0.1" floodColor="black" />
    //           </filter>
    //           <polygon
    //             x={x}
    //             y={-y}
    //             width={10}
    //             height={-10}
    //             fill={fillColor}
    //             stroke="black"
    //             strokeWidth="0.03"
    //             filter={`url(#${door.name}-shadow)`}
    //           />
    //         </g>
    //       );
    //     })}
    //   </svg>
    // </SVGOverlay>
  );
}
