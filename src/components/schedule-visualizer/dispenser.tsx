import { makeStyles } from '@material-ui/core';
import React from 'react';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { radiansToDegrees } from '../../util/calculation-helpers';
import { ResourcesContext } from '../../app-contexts';
import { ResourceDispenserConfigurationInterface } from '../../resource-manager-dispensers';
import ColorManager from './colors';
import { UpArrow } from './arrow';
import SvgText from './svg-text';

export interface DispenserProps {
  id?: string;
  colorManager: ColorManager;
  dispenser: ResourceDispenserConfigurationInterface;
  dispenserState?: RomiCore.DispenserState;
  footprint: number;
  onClick?(e: React.MouseEvent<SVGGElement>, dispenser: RomiCore.DispenserState): void;
}

const Dispenser = React.forwardRef(function(
  props: DispenserProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const { dispenser, footprint, dispenserState, onClick } = props;
  const resourcesContext = React.useContext(ResourcesContext);
  const classes = useStyles();
  return (
    <>
      <g
        ref={ref}
        className={`${classes.container}`}
        onClick={e => onClick && dispenserState && onClick(e, dispenserState)}
        transform={`translate(${dispenser.location.x} ${dispenser.location.y})
        rotate(${-(dispenser.location.yaw * 180) / Math.PI})`}
      >
        <g transform={`translate(${-footprint} ${-footprint})`}>
          <image
            href={'/assets/move_to_inbox_black_192x192.png'}
            height={footprint * 2}
            width={footprint * 2}
            onError={error => {
              console.error(
                'An error occurred while loading the image. Using the default image.',
                error,
              );
            }}
          />
        </g>
      </g>

      <SvgText
        id="dispenserName"
        x={dispenser.location.x}
        y={dispenser.location.y}
        className={classes.dispenserText}
        text={dispenser.name}
        targetWidth={footprint * 2.2}
      />
    </>
  );
});

export default Dispenser;

const useStyles = makeStyles(() => ({
  dispenserText: {
    dominantBaseline: 'central',
    textAnchor: 'middle',
    fontSize: '0.18px',
    fontWeight: 'bold',
    fill: 'white',
    /* 1 pixel black shadow to left, top, right and bottom */
    textShadow: '-1px 0 black, 0 1px black, 1px 0 black, 0 -1px black',
    pointerEvents: 'none',
  },

  container: {
    pointerEvents: 'visible',
    cursor: 'pointer',
  },
}));
