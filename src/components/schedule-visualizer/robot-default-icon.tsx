import { useTheme } from '@material-ui/core';
import React, { useState } from 'react';
import { RobotProps } from './robot';

type RobotDefaultIconProps = Omit<RobotProps, 'fleetName'>;

const RobotDefaultIcon = React.forwardRef(function(
  props: RobotDefaultIconProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const { robot, footprint, colorManager, inConflict } = props;
  const [robotColor, setRobotColor] = useState<string | null>(() =>
    colorManager.robotColorFromCache(robot.name, robot.model),
  );
  const theme = useTheme();

  React.useLayoutEffect(() => {
    if (robotColor) {
      return;
    }
    (async () => {
      setRobotColor(await colorManager.robotColor(robot.name, robot.model));
    })();
  }, [robot, robotColor, colorManager]);

  return (
    <>
      {!!robotColor && (
        <g>
          <defs>
            <radialGradient id="RobotDefaultIcon-shadow">
              <stop offset="70%" stop-color="#000000ff" />
              <stop offset="75%" stop-color="#00000080" />
              <stop offset="80%" stop-color="#00000060" />
              <stop offset="85%" stop-color="#00000030" />
              <stop offset="90%" stop-color="#00000018" />
              <stop offset="95%" stop-color="#00000008" />
              <stop offset="100%" stop-color="#00000000" />
            </radialGradient>
          </defs>
          <circle r={footprint * 1.3} fill="url(#RobotDefaultIcon-shadow)" />
          <circle r={footprint} fill={robotColor} />
          <line x2={footprint} stroke={theme.palette.common.black} strokeWidth="0.05" />
        </g>
      )}
    </>
  );
});

export default RobotDefaultIcon;
