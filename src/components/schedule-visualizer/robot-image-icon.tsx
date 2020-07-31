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

type RobotImageIconProps = Omit<RobotProps, 'colorManager'> & {
  iconPath: string;
  dispatchIconError: React.Dispatch<React.SetStateAction<boolean>>;
};

const RobotImageIcon = React.forwardRef(function(
  props: RobotImageIconProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const classes = useStyles();
  const { robot, footprint, iconPath, dispatchIconError, onClick } = props;
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
            onError={() => dispatchIconError(true)}
          />
        </g>
      )}
    </>
  );
});

export default RobotImageIcon;
