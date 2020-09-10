import { useTheme } from '@material-ui/core';
import React, { useState } from 'react';
import { RobotProps } from './robot';

const RobotDefaultIcon = React.forwardRef(function(
  props: RobotProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const { robot, footprint, colorManager, inConflict, fleetName } = props;
  const [robotColor, setRobotColor] = useState<string | null>(() =>
    colorManager.robotColorFromCache(fleetName, robot.name),
  );
  const theme = useTheme();
  React.useLayoutEffect(() => {
    if (robotColor) {
      return;
    }
    (async () => {
      setRobotColor(await colorManager.robotColor(fleetName, robot.name, robot.model));
      await colorManager.robotPrimaryColor(fleetName, robot.name, robot.model);
    })();
  }, [robot, robotColor, colorManager, fleetName]);

  return (
    <>
      {!!robotColor && (
        <g>
          <filter id={`${robot.name}-shadow`} x="-20%" y="-20%" width="140%" height="140%">
            <feDropShadow
              dx="0"
              dy="0"
              stdDeviation={footprint * 0.15}
              floodColor={inConflict ? theme.palette.error.main : theme.palette.common.black}
            />
          </filter>
          <circle
            r={footprint}
            fill={robotColor}
            filter={`url(${encodeURI(`#${robot.name}-shadow`)}`}
          />
          <line x2={footprint} stroke={theme.palette.common.black} strokeWidth="0.05" />
        </g>
      )}
    </>
  );
});

export default RobotDefaultIcon;
