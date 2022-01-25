import { styled } from '@mui/material';
import clsx from 'clsx';
import Debug from 'debug';
import React from 'react';
import { DefaultMarker } from './default-robot-marker';
import { ImageMarker } from './image-marker';

const debug = Debug('Map:RobotMarker');

const classes = {
  clickable: 'robot-marker-clickable',
};
const StyledG = styled('g')(() => ({
  [`& .${classes.clickable}`]: {
    pointerEvents: 'auto',
    cursor: 'pointer',
  },
}));

export interface RobotMarkerProps extends React.PropsWithRef<React.SVGProps<SVGGElement>> {
  cx: number;
  cy: number;
  r: number;
  color: string;
  inConflict?: boolean;
  iconPath?: string;
}

// TODO: Support rectangle markers?
export const RobotMarker = React.forwardRef(
  (
    { cx, cy, r, color, inConflict, iconPath, ...otherProps }: RobotMarkerProps,
    ref: React.Ref<SVGGElement>,
  ) => {
    debug('render');
    const [imageHasError, setImageHasError] = React.useState(false);
    const useImageMarker = !!iconPath && !imageHasError;
    const imageErrorHandler = React.useCallback(() => setImageHasError(true), []);

    return (
      <StyledG ref={ref} {...otherProps}>
        <g className={clsx(otherProps.onClick && classes.clickable)}>
          {useImageMarker ? (
            <ImageMarker
              cx={cx}
              cy={cy}
              r={r}
              // iconPath should always be truthy if useImageMarker is truthy due to above check.
              iconPath={iconPath!} // eslint-disable-line @typescript-eslint/no-non-null-assertion
              onError={imageErrorHandler}
              inConflict={inConflict}
            />
          ) : (
            <DefaultMarker cx={cx} cy={cy} r={r} color={color} inConflict={inConflict} />
          )}
        </g>
      </StyledG>
    );
  },
);

export default RobotMarker;
