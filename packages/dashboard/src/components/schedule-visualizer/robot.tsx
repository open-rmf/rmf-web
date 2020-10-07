import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React, { useState } from 'react';
import ColorManager from './colors';
import RobotDefaultIcon from './robot-default-icon';
import RobotImageIcon from './robot-image-icon';
import SvgText from './svg-text';
import { RobotResourceManager } from '../../resource-manager-robots';

const debug = Debug('ScheduleVisualizer:Robot');

const useStyles = makeStyles(() => ({
  robotText: {
    dominantBaseline: 'central',
    textAnchor: 'middle',
    fontSize: '0.18px',
    fontWeight: 'bold',
    fill: 'white',
    /* 1 pixel black shadow to left, top, right and bottom */
    textShadow: '-1px 0 black, 0 1px black, 1px 0 black, 0 -1px black',
    pointerEvents: 'none',
  },

  container: {
    pointerEvents: 'visible',
    cursor: 'pointer',
  },
}));

export interface RobotProps {
  robot: RomiCore.RobotState;
  colorManager: ColorManager;
  footprint: number;
  fleetName: string;
  robotHandler?: RobotResourceManager;
  inConflict?: boolean;
  onClick?(e: React.MouseEvent<SVGGElement>, fleetName: string, robot: RomiCore.RobotState): void;
}

const Robot = React.memo(
  React.forwardRef(function (props: RobotProps, ref: React.Ref<SVGGElement>): React.ReactElement {
    const classes = useStyles();
    const { robot, footprint, colorManager, fleetName, inConflict, robotHandler, onClick } = props;
    debug('render %s', robot.name);
    // The only image formats SVG software support are JPEG, PNG, and other SVG files.
    const [renderCustomIcon, setRenderCustomIcon] = useState({
      path: robotHandler ? robotHandler.getIconPath(fleetName) : null,
      error: false,
    });

    const handleLoadImageError = React.useCallback(() => {
      setRenderCustomIcon((previousVal) => {
        return { ...previousVal, error: true };
      });
    }, []);

    return (
      <>
        <g
          ref={ref}
          data-component="Robot"
          className={`${classes.container}`}
          aria-label={robot.name}
          onClick={(e) => onClick && onClick(e, fleetName, robot)}
          transform={`translate(${robot.location.x} ${-robot.location.y})
            rotate(${-(robot.location.yaw * 180) / Math.PI})`}
        >
          {!!renderCustomIcon.path && !renderCustomIcon.error ? (
            <RobotImageIcon
              iconPath={renderCustomIcon.path}
              robot={robot}
              footprint={footprint}
              dispatchIconError={handleLoadImageError}
              inConflict={inConflict}
              colorManager={colorManager}
              fleetName={fleetName}
            />
          ) : (
            <RobotDefaultIcon
              robot={robot}
              footprint={footprint}
              colorManager={colorManager}
              inConflict={inConflict}
              fleetName={fleetName}
            ></RobotDefaultIcon>
          )}
        </g>
        <SvgText
          id="robotName"
          text={robot.name}
          x={robot.location.x}
          y={-robot.location.y}
          targetWidth={footprint * 1.9}
          className={classes.robotText}
        />
      </>
    );
  }),
);

export default Robot;
