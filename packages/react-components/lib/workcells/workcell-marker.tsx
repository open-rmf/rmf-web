import { makeStyles } from '@material-ui/core';
import Debug from 'debug';
import React from 'react';
import { fromRmfCoords } from '../geometry-utils';
import SvgText from '../svg-text';
import WorkcellDefaultIcon from './default-marker-icon';

const debug = Debug('Dispensers:DispenserMarker');

const useStyles = makeStyles(() => ({
  text: {
    dominantBaseline: 'central',
    textAnchor: 'middle',
    fontSize: '0.18px',
    fontWeight: 'bold',
    fill: 'white',
    /* 1 pixel black shadow to left, top, right and bottom */
    textShadow: '-1px 0 black, 0 1px black, 1px 0 black, 0 -1px black',
    pointerEvents: 'none',
    userSelect: 'none',
  },

  clickable: {
    pointerEvents: 'auto',
    cursor: 'pointer',
  },
}));

export interface WorkcellMarkerProps extends Omit<React.SVGProps<SVGGElement>, 'onClick'> {
  guid: string;
  /**
   * Location of the dispenser in RMF coordinates.
   */
  location: [number, number];

  /**
   * radius of the dispenser.
   * default: 0.4
   */
  footprint?: number;
  iconPath?: string;
  onClick?(e: React.MouseEvent<SVGGElement>, guid: string): void;
}

export const WorkcellMarker = React.forwardRef(function (
  props: WorkcellMarkerProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const { guid, location: location_, footprint = 0.4, iconPath, onClick, ...otherProps } = props;
  debug(`render ${guid}`);
  const classes = useStyles();
  const [imageHasError, setImageHasError] = React.useState(false);
  const location = fromRmfCoords(location_);
  const useImageIcon = !!iconPath && !imageHasError;

  return (
    <g ref={ref} onClick={(e) => onClick && onClick(e, guid)} {...otherProps}>
      <g
        className={onClick ? classes.clickable : undefined}
        transform={`translate(${location[0]} ${location[1]})`}
      >
        {useImageIcon ? (
          <image
            href={iconPath}
            x={-footprint}
            y={-footprint}
            width={footprint * 2}
            height={footprint * 2}
            onError={() => setImageHasError(true)}
          />
        ) : (
          // the default marker's size is slightly smaller than the footprint
          <WorkcellDefaultIcon footprint={footprint * 1.4} />
        )}
        <rect
          x={-footprint}
          y={-footprint}
          width={footprint * 2}
          height={footprint * 2}
          fill="transparent"
        ></rect>
        <SvgText className={classes.text} text={guid} targetWidth={footprint * 2.2} />
      </g>
    </g>
  );
});

export default WorkcellMarker;
