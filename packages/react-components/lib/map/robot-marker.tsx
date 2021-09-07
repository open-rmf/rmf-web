import { makeStyles } from '@material-ui/core';
import clsx from 'clsx';
import Debug from 'debug';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { DefaultMarker } from './default-robot-marker';
import { ImageMarker } from './image-marker';

const debug = Debug('Map:RobotMarker');

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
        </g>
      </g>
    );
  },
);

export type RobotMarkerProps = React.ComponentPropsWithRef<typeof RobotMarker>;

export default RobotMarker;
