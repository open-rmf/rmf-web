import { makeStyles } from '@material-ui/core';
import { useTheme } from '@material-ui/core/styles';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React, { useMemo, useState, useEffect, useContext } from 'react';
import ColorManager from './colors';
import IconManager from '../../icons-manager';
import { IconContext } from '../../app-contexts';

const useStyles = makeStyles(() => ({
  robotMarker: {
    cursor: 'pointer',
    pointerEvents: 'auto',
  },
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
  const { robot, footprint, colorManager, onClick, inConflict } = props;
  const [robotColor, setRobotColor] = React.useState<string | null>(() =>
    colorManager.robotColorFromCache(robot.name, robot.model),
  );
  const [iconError, setIconError] = useState(false);
  const [renderDefaultIcon, setRenderDefaultIcon] = useState(false);

  const theme = useTheme();

  React.useLayoutEffect(() => {
    if (robotColor) {
      return;
    }
    (async () => {
      setRobotColor(await colorManager.robotColor(robot.name, robot.model));
    })();
  }, [robot, robotColor, colorManager]);

  // The only image formats SVG software support are JPEG, PNG, and other SVG files.
  const iconPath = useMemo(
    () => IconManager.getRobotIcon(iconContext, !!robot.model ? robot.model : robot.name),
    [iconContext, robot],
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
            transform={`translate(${robot.location.x} ${-robot.location.y}) 
            rotate(${-(robot.location.yaw * 180) / Math.PI}, ${footprint}, ${footprint})`}
            onClick={e => onClick && onClick(e, robot)}
          >
            <image
              href={iconPath}
              height={footprint * 2}
              width={footprint * 2}
              onError={() => setIconError(true)}
            />
          </g>
        )}
        {renderDefaultIcon && !!robotColor && (
          <g
            transform={`translate(${robot.location.x} ${-robot.location.y})
            rotate(${-(robot.location.yaw * 180) / Math.PI})`}
          >
            <filter id={`${robot.name}-shadow`} x="-20%" y="-20%" width="140%" height="140%">
              <feDropShadow
                dx="0"
                dy="0"
                stdDeviation={footprint * 0.15}
                floodColor={inConflict ? theme.palette.error.main : theme.palette.common.black}
              />
            </filter>
            <circle
              className={classes.robotMarker}
              onClick={e => onClick && onClick(e, robot)}
              r={footprint}
              fill={robotColor}
              filter={`url(#${robot.name}-shadow)`}
            />
            <line x2={footprint} stroke={theme.palette.common.black} strokeWidth="0.05" />
          </g>
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
