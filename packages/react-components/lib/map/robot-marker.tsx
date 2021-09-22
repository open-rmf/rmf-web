import { makeStyles } from '@material-ui/core';
import clsx from 'clsx';
import Debug from 'debug';
import React from 'react';
import { DefaultMarker } from './default-robot-marker';
import { ImageMarker } from './image-marker';

const debug = Debug('Map:RobotMarker');

const useStyles = makeStyles(() => ({
  clickable: {
    pointerEvents: 'auto',
    cursor: 'pointer',
  },
}));

export interface RobotMarkerProps extends React.PropsWithRef<React.SVGProps<SVGGElement>> {
  color: string;
  inConflict?: boolean;
  iconPath?: string;
}

export const RobotMarker = React.forwardRef(
  (
    { color, inConflict, iconPath, ...otherProps }: RobotMarkerProps,
    ref: React.Ref<SVGGElement>,
  ) => {
    debug('render');
    const [imageHasError, setImageHasError] = React.useState(false);
    const classes = useStyles();
    const useImageMarker = !!iconPath && !imageHasError;

    return (
      <g ref={ref} {...otherProps}>
        <g className={clsx(otherProps.onClick && classes.clickable)}>
          {useImageMarker ? (
            <ImageMarker
              // iconPath should always be truthy if useImageMarker is truthy due to above check.
              iconPath={iconPath!} // eslint-disable-line @typescript-eslint/no-non-null-assertion
              onError={() => setImageHasError(true)}
              inConflict={inConflict}
            />
          ) : (
            <DefaultMarker color={color} inConflict={inConflict} />
          )}
        </g>
      </g>
    );
  },
);

export default RobotMarker;
