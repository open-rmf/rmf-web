import { useTheme } from '@mui/material';
import { makeStyles } from '@mui/styles';
import Debug from 'debug';
import React from 'react';
import { fromRmfCoords } from '../geometry-utils';
import SvgText from '../svg-text';

const debug = Debug('Map:WorkcellMarker');

const DefaultIcon = (props: { footprint: number }): JSX.Element => {
  const { footprint } = props;
  const theme = useTheme();
  return (
    <svg
      xmlns="http://www.w3.org/2000/svg"
      x={-footprint}
      y={-footprint}
      width={footprint * 2}
      height={footprint * 2}
      viewBox="0 0 24 24"
    >
      {/* FIXME: The size is slightly smaller than the footprint */}
      <path
        fill={theme.palette.success.main}
        d="M19 3H4.99c-1.11 0-1.98.9-1.98 2L3 19c0 1.1.88 2 1.99 2H19c1.1 0 2-.9 2-2V5c0-1.1-.9-2-2-2zm0 12h-4c0 1.66-1.35 3-3 3s-3-1.34-3-3H4.99V5H19v10zm-3-5h-2V7h-4v3H8l4 4 4-4z"
      />
    </svg>
  );
};

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

export interface WorkcellMarkerProps extends React.PropsWithRef<React.SVGProps<SVGGElement>> {
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
}

export const WorkcellMarker = React.forwardRef(function (
  { guid, location: location_, footprint = 0.4, iconPath, ...otherProps }: WorkcellMarkerProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  debug(`render ${guid}`);
  const classes = useStyles();
  const [imageHasError, setImageHasError] = React.useState(false);
  const location = fromRmfCoords(location_);
  const useImageIcon = !!iconPath && !imageHasError;

  return (
    <g ref={ref} {...otherProps}>
      <g
        className={otherProps.onClick && classes.clickable}
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
          <DefaultIcon footprint={footprint * 1.4} />
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
