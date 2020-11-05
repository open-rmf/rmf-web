import { makeStyles } from '@material-ui/core';
import React from 'react';
import { joinClasses } from '../css-utils';
import SvgText from '../svg-text';
import DefaultMarkerIcon from './default-marker-icon';

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

export interface DispenserMarkerProps extends Omit<React.SVGProps<SVGGElement>, 'onClick'> {
  guid: string;

  /**
   * radius of the dispenser.
   * default: 0.4
   */
  footprint?: number;
  iconPath?: string;
  onClick?(e: React.MouseEvent<SVGGElement>, guid: string): void;
}

export const DispenserMarker = React.memo(
  React.forwardRef(function (
    props: DispenserMarkerProps,
    ref: React.Ref<SVGGElement>,
  ): React.ReactElement {
    const { guid, footprint = 0.4, iconPath, onClick, className, ...otherProps } = props;
    const classes = useStyles();
    const [useImageIcon, setUseImageIcon] = React.useState(!!iconPath);

    return (
      <g
        className={joinClasses(className, onClick ? classes.clickable : undefined)}
        onClick={(e) => onClick && onClick(e, guid)}
      >
        <g ref={ref} {...otherProps}>
          {useImageIcon ? (
            <image
              href={iconPath}
              x={-footprint}
              y={-footprint}
              width={footprint * 2}
              height={footprint * 2}
              onError={() => setUseImageIcon(false)}
            />
          ) : (
            // the default marker's size is slightly smaller than the footprint
            <DefaultMarkerIcon footprint={footprint * 1.4} />
          )}
          <rect
            x={-footprint}
            y={-footprint}
            width={footprint * 2}
            height={footprint * 2}
            fill="transparent"
          ></rect>
        </g>
        <SvgText className={classes.text} text={guid} targetWidth={footprint * 2.2} />
      </g>
    );
  }),
);

export default DispenserMarker;
