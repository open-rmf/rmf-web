import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React, { useState, useContext } from 'react';
import ColorManager from './colors';
import ResourceManager from '../../resource-manager';
import { ResourcesContext } from '../../app-contexts';
import RobotDefaultIcon from './robot-default-icon';
import RobotImageIcon from './robot-image-icon';

const useStyles = makeStyles(() => ({
  robotText: {
    dominantBaseline: 'central',
    textAnchor: 'middle',
    fontSize: '0.18px',
    fontWeight: 'bold',
    fill: 'white',
    /* 1 pixel black shadow to left, top, right and bottom */
    textShadow: '-1px 0 black, 0 1px black, 1px 0 black, 0 -1px black',
  },
}));

export interface RobotProps {
  robot: RomiCore.RobotState;
  colorManager: ColorManager;
  footprint: number;
  fleetName: string;
  onClick?(e: React.MouseEvent<SVGGElement>, robot: RomiCore.RobotState): void;
  inConflict?: boolean;
}

const Robot = React.forwardRef(function(
  props: RobotProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const resourcesContext = useContext(ResourcesContext);
  const classes = useStyles();
  const { robot, footprint, colorManager, fleetName, inConflict, onClick } = props;
  // The only image formats SVG software support are JPEG, PNG, and other SVG files.
  const [renderCustomIcon, setRenderCustomIcon] = useState({
    path: ResourceManager.getRobotIconPath(resourcesContext, fleetName),
    error: false,
  });

  return (
    <>
      <g
        ref={ref}
        data-component="Robot"
        aria-label={robot.name}
        onClick={e => onClick && onClick(e, robot)}
      >
        {!!renderCustomIcon.path && !renderCustomIcon.error ? (
          <RobotImageIcon
            iconPath={renderCustomIcon.path}
            robot={robot}
            footprint={footprint}
            dispatchIconError={setRenderCustomIcon}
            inConflict={inConflict}
            onClick={onClick}
          />
        ) : (
          <RobotDefaultIcon
            robot={robot}
            footprint={footprint}
            colorManager={colorManager}
            onClick={onClick}
            inConflict={inConflict}
          ></RobotDefaultIcon>
        )}
        <text
          id="robotName"
          x={robot.location.x}
          y={-robot.location.y}
          className={classes.robotText}
        >
          {robot.name.substring(0, 8)}
        </text>
      </g>
    </>
  );
});

export default Robot;
