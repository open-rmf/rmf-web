import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React, { useMemo, useState, useEffect, useContext } from 'react';
import ColorManager from './colors';
import IconManager from '../../icons-manager';
import { IconContext } from '../../app-contexts';
import RobotDefaultIcon from './robot-default-icon';
import { transformMiddleCoordsOfRectToSVGBeginPoint } from '../../util/angle-calculation';

const useStyles = makeStyles(() => ({
  robotText: {
    dominantBaseline: 'central',
    textAnchor: 'middle',
    fontSize: '0.18px',
    fontWeight: 'bold',
    fill: 'white',
  },
  robotImg: {
    transformOrigin: 'center',
  },
  robotImgContainer: {
    pointerEvents: 'visible',
  },
}));

export interface RobotProps {
  robot: RomiCore.RobotState;
  footprint: number;
  colorManager: ColorManager;
  onClick?(e: React.MouseEvent<SVGGElement>, robot: RomiCore.RobotState): void;
  inConflict?: boolean;
}

const Robot = React.forwardRef(function(
  props: RobotProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const iconContext = useContext(IconContext);
  const classes = useStyles();
  const { robot, footprint, colorManager, onClick } = props;
  const [iconError, setIconError] = useState(false);
  const [renderDefaultIcon, setRenderDefaultIcon] = useState(false);

  // The only image formats SVG software support are JPEG, PNG, and other SVG files.
  const iconPath = useMemo(
    () => IconManager.getRobotIcon(iconContext, !!robot.model ? robot.model : robot.name),
    [iconContext, robot],
  );

  const [imgIconWidth, imgIconHeigth] = useMemo(() => [footprint * 2, footprint * 2], [footprint]);

  const { x: topVerticeX, y: topVerticeY } = transformMiddleCoordsOfRectToSVGBeginPoint(
    robot.location.x,
    robot.location.y,
    imgIconWidth,
    imgIconHeigth,
  );

  useEffect(() => {
    setRenderDefaultIcon(!iconPath || !!iconError);
  }, [iconPath, iconError]);

  return (
    <>
      <g
        ref={ref}
        data-component="Robot"
        aria-label={robot.name}
        onClick={e => onClick && onClick(e, robot)}
      >
        {!!iconPath && !iconError && (
          <g
            className={classes.robotImgContainer}
            transform={`translate(${topVerticeX} ${-topVerticeY}) 
            rotate(${-(robot.location.yaw * 180) / Math.PI}, ${footprint}, ${footprint})`}
            onClick={e => onClick && onClick(e, robot)}
          >
            <image
              href={iconPath}
              height={imgIconHeigth}
              width={imgIconWidth}
              onError={() => setIconError(true)}
            />
          </g>
        )}
        {renderDefaultIcon && (
          <RobotDefaultIcon
            robot={robot}
            footprint={footprint}
            colorManager={colorManager}
            onClick={onClick}
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
