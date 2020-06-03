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
    colorManager.robotColorFromCache(robot.name, robot.model),
  );

  const nose = React.useMemo(
    () => [
      robot.location.x + Math.cos(robot.location.yaw) * footprint,
      robot.location.y + Math.sin(robot.location.yaw) * footprint,
    ],
    [robot, footprint],
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
    <g ref={ref} onClick={e => onClick && onClick(e, robot)}>
      {robotColor && (
        <React.Fragment>
          <filter id={`${robot.name}-shadow`} x="-20%" y="-20%" width="140%" height="140%">
            <feDropShadow dx="0" dy="0" stdDeviation={footprint * 0.15} floodColor="black" />
          </filter>
          <circle
            className={classes.robotMarker}
            onClick={e => onClick && onClick(e, robot)}
            cx={robot.location.x}
            cy={-robot.location.y}
            r={footprint}
            fill={robotColor}
            filter={`url(#${robot.name}-shadow)`}
          />
          <line
            x1={robot.location.x}
            y1={-robot.location.y}
            x2={nose[0]}
            y2={-nose[1]}
            stroke="black"
            strokeWidth="0.05"
          />
          <text
            id="robotName"
            className={classes.robotText}
            x={robot.location.x}
            y={-robot.location.y}
          >
            {robot.name.substring(0, 8)}
          </text>
        </React.Fragment>
      )}
    </g>
  );
});

export default Robot;
