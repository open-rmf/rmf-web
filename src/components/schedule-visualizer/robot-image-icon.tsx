import { makeStyles } from '@material-ui/core';
import React, { useMemo } from 'react';
import { transformMiddleCoordsOfRectToSVGBeginPoint } from '../../util/calculation-helpers';
import { RobotProps } from './robot';

const useStyles = makeStyles(() => ({
  robotImg: {
    transformOrigin: 'center',
  },
  robotImgContainer: {
    pointerEvents: 'visible',
  },
}));

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
  const classes = useStyles();
  const { robot, footprint, iconPath, dispatchIconError, onClick } = props;
  // Hardcoded delta of 2 to set the size of the robot. With the double size of the footprint, we achieved a similar size to the robot default svg icon.
  const [imgIconWidth, imgIconHeigth] = useMemo(() => [footprint * 2, footprint * 2], [footprint]);

  const { x: topVerticeX, y: topVerticeY } = transformMiddleCoordsOfRectToSVGBeginPoint(
    robot.location.x,
    robot.location.y,
    imgIconWidth,
    imgIconHeigth,
  );

  return (
    <>
      {!!iconPath && (
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
      )}
    </>
  );
});

export default RobotImageIcon;
