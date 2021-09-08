import { makeStyles } from '@material-ui/core';
import clsx from 'clsx';
import Debug from 'debug';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { DefaultMarker } from './default-robot-marker';
import { ImageMarker } from './image-marker';
import { NameLabel } from './marker-label';

const debug = Debug('Map:RobotMarker');

const useStyles = makeStyles(() => ({
  clickable: {
    pointerEvents: 'auto',
    cursor: 'pointer',
  },
}));

interface _RobotMarkerProps extends React.PropsWithoutRef<React.SVGProps<SVGGElement>> {
  fleet: string;
  name: string;
  model: string;
  state: RmfModels.RobotState;
  inConflict?: boolean;
  color: string;
  iconPath?: string;
}

export const RobotMarker = React.forwardRef(
  (
    { color, iconPath, fleet, name, inConflict, className, ...otherProps }: _RobotMarkerProps,
    ref: React.Ref<SVGGElement>,
  ) => {
    debug(`render ${fleet}/${name}`);
    const [imageHasError, setImageHasError] = React.useState(false);
    const classes = useStyles();
    const useImageMarker = !!iconPath && !imageHasError;
    const labelAnchorX = 0.707106781187; // cos(45°)
    const labelAnchorY = -0.707106781187; // sin(-45°)

    const isMounted = React.useRef(true);
    React.useEffect(() => {
      return () => {
        isMounted.current = false;
      };
    }, []);

    return (
      <g
        ref={ref}
        className={clsx(otherProps.onClick && classes.clickable, className)}
        {...otherProps}
      >
        <g>
          {useImageMarker && iconPath ? (
            <ImageMarker
              iconPath={iconPath}
              onError={() => setImageHasError(true)}
              inConflict={inConflict}
            />
          ) : (
            <DefaultMarker color={color} inConflict={inConflict} />
          )}
          <NameLabel
            anchorX={labelAnchorX}
            anchorY={labelAnchorY}
            text={name}
            transform="scale(12)"
          />
        </g>
      </g>
    );
  },
);

export type RobotMarkerProps = React.ComponentPropsWithRef<typeof RobotMarker>;

export default RobotMarker;
