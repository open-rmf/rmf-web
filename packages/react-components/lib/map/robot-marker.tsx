import { makeStyles } from '@material-ui/core';
import Debug from 'debug';
import React from 'react';
import { SvgText } from '../svg-text';
import { fromRmfCoords, fromRmfYaw } from '../geometry-utils';
import { BaseRobotMarkerProps } from './base-robot-marker';
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

export interface RobotMarkerProps extends React.PropsWithRef<BaseRobotMarkerProps> {
  color: string;
  iconPath?: string;
}

export const RobotMarker = React.forwardRef(
  (
    {
      color,
      iconPath,
      fleet,
      name,
      model,
      footprint,
      state,
      inConflict,
      translate = true,
      ...otherProps
    }: RobotMarkerProps,
    ref: React.Ref<SVGGElement>,
  ) => {
    debug(`render ${fleet}/${name}`);
    const [imageHasError, setImageHasError] = React.useState(false);
    const classes = useStyles();
    const pos = fromRmfCoords([state.location.x, state.location.y]);
    const yaw = (fromRmfYaw(state.location.yaw) / Math.PI) * 180;
    const useImageMarker = !!iconPath && !imageHasError;

    const translateTransform = translate ? `translate(${pos[0]} ${pos[1]})` : undefined;

    const isMounted = React.useRef(true);
    React.useEffect(() => {
      return () => {
        isMounted.current = false;
      };
    }, []);

    return (
      <g ref={ref} {...otherProps}>
        <g transform={translateTransform}>
          <g className={otherProps.onClick && classes.clickable} transform={`rotate(${yaw})`}>
            {useImageMarker && iconPath ? (
              <ImageMarker
                iconPath={iconPath}
                onError={() => setImageHasError(true)}
                fleet={fleet}
                name={name}
                model={model}
                footprint={footprint}
                state={state}
                inConflict={inConflict}
              />
            ) : (
              <DefaultMarker
                color={color}
                fleet={fleet}
                name={name}
                model={model}
                footprint={footprint}
                state={state}
                inConflict={inConflict}
              />
            )}
          </g>
          <SvgText text={name} targetWidth={footprint * 1.9} className={classes.text} />
        </g>
      </g>
    );
  },
);

export default RobotMarker;
