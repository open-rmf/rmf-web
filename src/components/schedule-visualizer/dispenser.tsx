import { makeStyles } from '@material-ui/core';
import React from 'react';
import { radiansToDegrees } from '../../util/calculation-helpers';
import { ResourcesContext } from '../../app-contexts';
import { ResourceDispenserConfigurationInterface } from '../../resource-manager-dispensers';
import ColorManager from './colors';

export interface DispenserProps {
  id?: string;
  colorManager: ColorManager;
  dispenser: ResourceDispenserConfigurationInterface;
  footprint: number;
  onClick?(): void;
}

const Dispenser = React.forwardRef(function(
  props: DispenserProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const { id, dispenser, footprint, onClick } = props;
  const resourcesContext = React.useContext(ResourcesContext);
  const classes = useStyles();
  return (
    <>
      <g ref={ref} id={id} onClick={onClick}>
        <rect
          className={`${classes.liftMarker}`}
          width={footprint * 2}
          height={footprint * 2}
          x={dispenser.location.x}
          y={dispenser.location.y}
          rx="0.1"
          ry="0.1"
          transform={`rotate(${radiansToDegrees(dispenser.location.yaw)}, ${dispenser.location.x},${
            dispenser.location.y
          })`}
        />
        <text
          id="robotName"
          x={dispenser.location.x}
          y={dispenser.location.y}
          className={classes.dispenserText}
        >
          {dispenser.name.substring(0, 8)}
        </text>
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
  dispenserText: {
    dominantBaseline: 'central',
    textAnchor: 'middle',
    fontSize: '0.18px',
    fontWeight: 'bold',
    fill: 'white',
    /* 1 pixel black shadow to left, top, right and bottom */
    textShadow: '-1px 0 black, 0 1px black, 1px 0 black, 0 -1px black',
  },
  liftText: {
    dominantBaseline: 'central',
    textAnchor: 'middle',
    fontSize: '0.18px',
    fontWeight: 'bold',
  },
}));
