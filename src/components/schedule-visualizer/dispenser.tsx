import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import Door from './door/door';
import { UpArrow, DownArrow } from './arrow';
import {
  radiansToDegrees,
  transformMiddleCoordsOfRectToSVGBeginPoint,
} from '../../util/calculation-helpers';

export interface DispenserProps {
  id?: string;
  currentFloor: string;
  position: RomiCore.Lift;
  liftState?: RomiCore.LiftState;
  onClick?(e: React.MouseEvent<SVGGElement>, lift: RomiCore.Lift): void;
}

const Dispenser = React.forwardRef(function(
  props: DispenserProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  return (
    <>
      <g ref={ref} id={id} onClick={e => onClick && onClick(e, lift)}>
        <rect
          className={`${classes.liftMarker} ${classes.lift} ${liftStyle}`}
          width={width}
          height={depth}
          x={topVerticeX}
          y={contextTopVerticeY}
          rx="0.1"
          ry="0.1"
          transform={`rotate(${radiansToDegrees(ref_yaw)}, ${x},${contextY})`}
        />
      </g>
    </>
  );
});

export default Dispenser;

const useStyles = makeStyles(() => ({
  liftMarker: {
    cursor: 'pointer',
    pointerEvents: 'auto',
  },
  lift: {
    strokeWidth: '0.2',
  },
  liftOnCurrentFloor: {
    fill: 'green',
    opacity: '70%',
  },
  liftMoving: {
    fill: 'grey',
    opacity: '70%',
  },
  unknownLift: {
    fill: '#3d3c3c',
    opacity: '80%',
  },
  emergency: {
    fill: 'red',
    opacity: '80%',
  },
  fire: {
    fill: '#ff562a',
    opacity: '80%',
  },
  offLine: {
    fill: 'yellow',
    opacity: '80%',
  },
  humanMode: {
    fill: '#90dfef',
    opacity: '80%',
  },
  liftText: {
    dominantBaseline: 'central',
    textAnchor: 'middle',
    fontSize: '0.18px',
    fontWeight: 'bold',
  },
}));
