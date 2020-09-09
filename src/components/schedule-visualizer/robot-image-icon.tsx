import { useTheme } from '@material-ui/core';
import React, { useMemo } from 'react';
import { RobotProps } from './robot';

type RobotImageIconProps = Omit<RobotProps, 'colorManager' | 'fleetName'> & {
  iconPath: string;
  dispatchIconError: React.Dispatch<
    React.SetStateAction<{
      path: string | null;
      error: boolean;
    }>
  >;
};

const RobotImageIcon = React.forwardRef(function(
  props: RobotImageIconProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const { robot, footprint, iconPath, dispatchIconError, inConflict } = props;
  const theme = useTheme();
  // The default icon uses footprint as the radius, so we * 2 here because the width/height
  // is in a square. With the double size of the footprint, we achieved a similar
  // size to the robot default svg icon.
  const [imgIconWidth, imgIconHeigth] = useMemo(() => [footprint * 2, footprint * 2], [footprint]);

  return (
    <>
      {!!iconPath && (
        <g>
          <defs>
            <radialGradient id="RobotImageIcon-shadow">
              <stop offset="0%" stop-color="#00000080" />
              <stop offset="70%" stop-color="#00000040" />
              <stop offset="90%" stop-color="#00000010" />
              <stop offset="100%" stop-color="#00000000" />
            </radialGradient>
          </defs>
          <circle r={footprint * 1.3} fill="url(#RobotImageIcon-shadow)" />
          <g transform={`translate(${-footprint} ${-footprint})`}>
            <image
              href={iconPath}
              height={imgIconHeigth}
              width={imgIconWidth}
              // filter={`url(#${robot.name}-shadow)`}
              onError={error => {
                console.error(
                  'An error occurred while loading the image. Using the default image.',
                  error,
                );
                return dispatchIconError(previousVal => {
                  return { ...previousVal, error: true };
                });
              }}
            />
          </g>
        </g>
      )}
    </>
  );
});

export default RobotImageIcon;
