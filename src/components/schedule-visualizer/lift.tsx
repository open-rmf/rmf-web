import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import { RomiCoreLift } from './lift-overlay';

export interface LiftProps {
  currentLevel: string;
  // TODO: this should be replaced by RomiCore.Lift once we addressed this
  // https://github.com/osrf/romi-js-core-interfaces/issues/4
  lift: RomiCoreLift;
  liftState?: RomiCore.LiftState;
  onClick?(e: React.MouseEvent<SVGGElement>, lift: RomiCoreLift): void;
}

const Lift = React.forwardRef(function(
  props: LiftProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const { lift, onClick, liftState } = props;
  const { width, depth, ref_x: x, ref_y: y } = lift;
  const currentFloor = liftState?.current_floor;
  const currentMode = liftState?.current_mode;
  const doorState = liftState?.door_state;
  const motionState = liftState?.motion_state;

  const classes = useStyles();
  return (
    <>
      <g ref={ref} onClick={e => onClick && onClick(e as any, lift)}>
        <rect
          className={`${classes.liftMarker} ${classes.lift}`}
          width={width}
          height={depth}
          x={x}
          y={y}
        />
      </g>
    </>
  );
});

export default Lift;

const useStyles = makeStyles(() => ({
  liftMarker: {
    cursor: 'pointer',
    pointerEvents: 'auto',
  },
  lift: {
    strokeWidth: '0.2',
  },
}));
