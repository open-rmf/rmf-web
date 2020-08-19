import { makeStyles, useTheme } from '@material-ui/core';
import React, { useState } from 'react';
import { RobotProps } from './robot';

const useStyles = makeStyles(() => ({
  robotMarker: {
    cursor: 'pointer',
    pointerEvents: 'auto',
  },
}));

type RobotDefaultIconProps = Omit<RobotProps, 'fleetName'>;

const RobotDefaultIcon = React.forwardRef(function(
  props: RobotDefaultIconProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const classes = useStyles();
  const { robot, footprint, colorManager, inConflict, onClick } = props;
  const [robotColor, setRobotColor] = useState<string | null>(() =>
    colorManager.robotColorFromCache(robot.name),
  );
  const theme = useTheme();
  React.useLayoutEffect(() => {
    if (robotColor) {
      return;
    }
    (async () => {
      setRobotColor(await colorManager.robotColor(robot.name, robot.model));
    })();
    (async () => {
      await colorManager.robotTrajectoryColor(robot.name, robot.model);
    })();
  }, [robot, robotColor, colorManager]);

  return (
    <>
      {!!robotColor && (
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
    </>
  );
});

export default RobotDefaultIcon;
