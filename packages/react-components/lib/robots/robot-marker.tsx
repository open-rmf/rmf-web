import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import { SvgText } from '..';
import { fromRmfCoords, fromRmfYaw } from '../geometry-utils';
import { DefaultMarker } from './default-marker';
import { ImageMarker } from './image-marker';

const debug = Debug('Robots:RobotMarker');

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

export interface RobotMarkerProps extends Omit<React.SVGAttributes<SVGGElement>, 'onClick'> {
  robot: RomiCore.RobotState;
  footprint: number;
  fleetName: string;
  iconPath?: string;
  /**
   * Whether the component should perform a translate transform to put it inline with the position
   * in RMF.
   *
   * default: true
   */
  translate?: boolean;
  variant?: 'normal' | 'inConflict';
  onClick?(event: React.MouseEvent, fleet: string, robot: RomiCore.RobotState): void;
}

/**
 * Contexts: ColorContext
 */
export const RobotMarker = React.forwardRef(
  (props: RobotMarkerProps, ref: React.Ref<SVGGElement>) => {
    // some props are not used but have to be declared to correctly set `otherProps`
    const {
      robot,
      footprint,
      fleetName,
      iconPath,
      // eslint-disable-next-line @typescript-eslint/no-unused-vars
      variant,
      translate = true,
      onClick,
      ...otherProps
    } = props;
    debug(`render ${robot.name}`);
    const [useImageMarker, setUseImageMarker] = React.useState(!!iconPath);
    const classes = useStyles();
    const pos = fromRmfCoords([robot.location.x, robot.location.y]);
    const yaw = (fromRmfYaw(robot.location.yaw) / Math.PI) * 180;

    const translateTransform = translate ? `translate(${pos[0]} ${pos[1]})` : undefined;

    return (
      <g ref={ref} onClick={(ev) => onClick && onClick(ev, fleetName, robot)} {...otherProps}>
        <g transform={translateTransform}>
          <g className={classes.clickable} aria-label={robot.name} transform={`rotate(${yaw})`}>
            {useImageMarker && iconPath ? (
              <ImageMarker
                {...props}
                iconPath={iconPath}
                onError={() => setUseImageMarker(false)}
              />
            ) : (
              <DefaultMarker {...props} />
            )}
          </g>
          <SvgText text={robot.name} targetWidth={footprint * 1.9} className={classes.text} />
        </g>
      </g>
    );
  },
);

export default RobotMarker;
