import { useTheme, styled } from '@mui/material';
import Debug from 'debug';
import React from 'react';

const debug = Debug('Map:WorkcellMarker');

export interface WorkcellMarkerProps extends React.PropsWithRef<React.SVGProps<SVGGElement>> {
  cx: number;
  cy: number;
  size: number;
  /**
   * Image must be 1x1 aspect ratio.
   */
  iconPath?: string;
}

const DefaultIcon = ({ cx, cy, size }: WorkcellMarkerProps): JSX.Element => {
  const theme = useTheme();
  return (
    <svg
      xmlns="http://www.w3.org/2000/svg"
      // The size of the svg is slightly smaller than the footprint.
      x={cx - size * 0.7}
      y={cy - size * 0.7}
      width={size * 1.4}
      height={size * 1.4}
      viewBox="0 0 24 24"
    >
      <path
        fill={theme.palette.success.main}
        d="M19 3H4.99c-1.11 0-1.98.9-1.98 2L3 19c0 1.1.88 2 1.99 2H19c1.1 0 2-.9 2-2V5c0-1.1-.9-2-2-2zm0 12h-4c0 1.66-1.35 3-3 3s-3-1.34-3-3H4.99V5H19v10zm-3-5h-2V7h-4v3H8l4 4 4-4z"
      />
    </svg>
  );
};

const classes = {
  text: 'workcell-marker-text',
  clickable: 'workcell-marker-clickable',
};
const StyledG = styled('g')(() => ({
  [`& .${classes.text}`]: {
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
  [`& .${classes.clickable}`]: {
    pointerEvents: 'auto',
    cursor: 'pointer',
  },
}));

// TODO: Support rectangle marker?
export const WorkcellMarker = React.forwardRef(function (
  props: WorkcellMarkerProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  debug('render');
  const { cx, cy, size, iconPath, ...otherProps } = props;
  const [imageHasError, setImageHasError] = React.useState(false);
  const useImageIcon = !!iconPath && !imageHasError;

  return (
    <StyledG ref={ref} {...otherProps}>
      <g className={otherProps.onClick && classes.clickable}>
        {useImageIcon ? (
          <image
            href={iconPath}
            x={cx - size / 2}
            y={cy - size / 2}
            width={size}
            height={size}
            onError={() => setImageHasError(true)}
          />
        ) : (
          // the default marker's size is slightly smaller than the footprint
          <DefaultIcon {...props} />
        )}
        <rect
          x={cx - size / 2}
          y={cy - size / 2}
          width={size}
          height={size}
          fill="transparent"
        ></rect>
      </g>
    </StyledG>
  );
});

export default WorkcellMarker;
