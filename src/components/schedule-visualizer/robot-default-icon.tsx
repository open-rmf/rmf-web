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
  const shadowImpl = React.useContext(TmpShadowContext);

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
            <radialGradient id="shadow-gradient">
              <stop offset="70%" stop-color="#000000ff" />
              <stop offset="75%" stop-color="#00000080" />
              <stop offset="80%" stop-color="#00000060" />
              <stop offset="85%" stop-color="#00000030" />
              <stop offset="90%" stop-color="#00000018" />
              <stop offset="95%" stop-color="#00000008" />
              <stop offset="100%" stop-color="#00000000" />
            </radialGradient>
            <filter id="shadow-filter" x="-20%" y="-20%" width="140%" height="140%">
              <feDropShadow
                dx="0"
                dy="0"
                stdDeviation={footprint * 0.15}
                floodColor={inConflict ? theme.palette.error.main : theme.palette.common.black}
              />
            </filter>
          </defs>
          {shadowImpl === 'gradient' && <circle r={footprint * 1.3} fill="url(#shadow-gradient)" />}
          <circle
            r={footprint}
            fill={robotColor}
            filter={shadowImpl === 'filter' ? 'url(#shadow-filter)' : undefined}
          />
          <line x2={footprint} stroke={theme.palette.common.black} strokeWidth="0.05" />
        </g>
      )}
    </>
  );
});

export default RobotDefaultIcon;

// TODO: For demo purposes, remove once we decide which implementation to go for.
export const TmpShadowContext = React.createContext<'gradient' | 'filter'>('gradient');
