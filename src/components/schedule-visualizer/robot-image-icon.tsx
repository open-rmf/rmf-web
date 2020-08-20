import { makeStyles, useTheme } from '@material-ui/core';
import React, { useMemo, useState } from 'react';
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

type RobotImageIconProps = Omit<RobotProps, 'fleetName'> & {
  iconPath: string;
  dispatchIconError: React.Dispatch<
    React.SetStateAction<{
      path: string | null;
      error: boolean;
    }>
  >;
};

// const robotImageIconColor = async (path: string) => {
//   const imgHolder = new Image(400, 400);
//   imgHolder.src = path;

//   await new Promise((resolve, reject) => {
//     imgHolder.onload = () => resolve()
//     imgHolder.onerror = err => reject(err)
//   })

//   const canvas = document.createElement('canvas');
//   const context = canvas.getContext('2d')
//   context?.drawImage(imgHolder, 0, 0, imgHolder.width, imgHolder.height);
//   const data = context?.getImageData(100,100,1,1).data;
//   // returns in rbga format
//   return `rbga(${data?[0]: 0}, ${data?[1]: 0}, ${data?[2]: 0}, ${data?[3]: 0})`
// }

const RobotImageIcon = React.forwardRef(function(
  props: RobotImageIconProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const classes = useStyles();
  const {
    robot,
    footprint,
    iconPath,
    dispatchIconError,
    onClick,
    inConflict,
    colorManager,
  } = props;

  const theme = useTheme();
  // The default icon uses footprint as the radius, so we * 2 here because the width/height
  // is in a square. With the double size of the footprint, we achieved a similar
  // size to the robot default svg icon.
  const [imgIconWidth, imgIconHeigth] = useMemo(() => [footprint * 2, footprint * 2], [footprint]);
  const [robotColor, setRobotColor] = useState<string | null>(() =>
    colorManager.robotColorFromCache(robot.name),
  );

  React.useLayoutEffect(() => {
    if (robotColor) {
      return;
    }
    (async () => {
      setRobotColor(await colorManager.robotImageIconColor(iconPath, robot.name));
      // const test = colorManager.robotImageIconColor(iconPath);
      await colorManager.robotTrajectoryColor(robot.name, robot.model);
    })();
  }, [robot, robotColor, colorManager]);

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
          <filter id={`${robot.name}-shadow`} x="-20%" y="-20%" width="140%" height="140%">
            <feDropShadow
              dx="0"
              dy="0"
              stdDeviation={footprint * 0.15}
              floodColor={inConflict ? theme.palette.error.main : theme.palette.common.black}
            />
          </filter>
          <image
            href={iconPath}
            height={imgIconHeigth}
            width={imgIconWidth}
            filter={`url(#${robot.name}-shadow)`}
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
