import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import { SvgText } from '..';
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
  container: {
    pointerEvents: 'visible',
    cursor: 'pointer',
  },
}));

export interface RobotMarkerProps extends Omit<React.SVGAttributes<SVGGElement>, 'onClick'> {
  robot: RomiCore.RobotState;
  footprint: number;
  fleetName: string;
  iconPath?: string;
  variant?: 'normal' | 'inConflict';
  onClick?(event: React.MouseEvent, fleet: string, robot: RomiCore.RobotState): void;
}

/**
 * Contexts: ColorContext
 */
export const RobotMarker = React.memo(
  React.forwardRef((props: RobotMarkerProps, ref: React.Ref<SVGGElement>) => {
    // some props are not used but have to be declared to correctly set `otherProps`
    // eslint-disable-next-line @typescript-eslint/no-unused-vars
    const { robot, footprint, fleetName, iconPath, variant, onClick, ...otherProps } = props;
    debug(`render ${robot.name}`);
    const [useImageMarker, setUseImageMarker] = React.useState(!!iconPath);
    const classes = useStyles();

    return (
      <g>
        <g
          ref={ref}
          className={classes.container}
          aria-label={robot.name}
          transform={`translate(${robot.location.x} ${-robot.location.y})
            rotate(${-(robot.location.yaw * 180) / Math.PI})`}
          onClick={(ev) => onClick && onClick(ev, fleetName, robot)}
          {...otherProps}
        >
          {useImageMarker && iconPath ? (
            <ImageMarker {...props} iconPath={iconPath} onError={() => setUseImageMarker(false)} />
          ) : (
            <DefaultMarker {...props} />
          )}
        </g>
        <SvgText
          text={robot.name}
          x={robot.location.x}
          y={-robot.location.y}
          targetWidth={footprint * 1.9}
          className={classes.text}
        />
      </g>
    );
  }),
);

export default RobotMarker;
