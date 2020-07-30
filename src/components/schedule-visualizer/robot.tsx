import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import ColorManager from './colors';

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
}));

export interface RobotProps {
  robot: RomiCore.RobotState;
  footprint: number;
  colorManager: ColorManager;
  onClick?(e: React.MouseEvent<SVGGElement>, robot: RomiCore.RobotState): void;
}

const Robot = React.forwardRef(function(
  props: RobotProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const classes = useStyles();
  const { robot, footprint, colorManager, onClick } = props;
  const [robotColor, setRobotColor] = React.useState<string | null>(() =>
    colorManager.robotColorFromCache(robot.name),
  );

  React.useLayoutEffect(() => {
    if (robotColor) {
      return;
    }
    (async () => {
      setRobotColor(await colorManager.robotColor(robot.name, robot.model));
    })();
  }, [robot, robotColor, colorManager]);

  return (
    <g
      ref={ref}
      data-component="Robot"
      aria-label={robot.name}
      onClick={e => onClick && onClick(e, robot)}
    >
      {robotColor && (
        <>
          <g
            transform={`translate(${robot.location.x} ${-robot.location.y})
            rotate(${-(robot.location.yaw * 180) / Math.PI})`}
          >
            <filter id={`${robot.name}-shadow`} x="-20%" y="-20%" width="140%" height="140%">
              <feDropShadow dx="0" dy="0" stdDeviation={footprint * 0.15} floodColor="black" />
            </filter>
            <circle
              className={classes.robotMarker}
              onClick={e => onClick && onClick(e, robot)}
              r={footprint}
              fill={robotColor}
              filter={`url(#${robot.name}-shadow)`}
            />
            <line x2={footprint} stroke="black" strokeWidth="0.05" />
          </g>
          <text
            id="robotName"
            x={robot.location.x}
            y={-robot.location.y}
            className={classes.robotText}
          >
            {robot.name.substring(0, 8)}
          </text>
        </>
      )}
    </g>
  );
});

export default Robot;
