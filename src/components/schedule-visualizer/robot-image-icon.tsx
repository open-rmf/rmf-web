import React, { SVGProps, useMemo, useState } from 'react';
import { uniqueId } from '../../util/css-utils';
import ImageIcon from './image-icon';
import { RobotProps } from './robot';

type RobotImageIconProps = RobotProps & {
  iconPath: string;
  dispatchIconError?(): void;
};

/**
 *
 * @param color MUST be in hex notation without alpha channel. e.g. #123456
 */
function makeGradientShadow(
  color: string,
): React.FunctionComponent<SVGProps<SVGRadialGradientElement>> {
  return (props: SVGProps<SVGRadialGradientElement>) => (
    <radialGradient {...props}>
      <stop offset="0%" stopColor={`${color}80`} />
      <stop offset="70%" stopColor={`${color}40`} />
      <stop offset="90%" stopColor={`${color}10`} />
      <stop offset="100%" stopColor={`${color}00`} />
    </radialGradient>
  );
}

const RobotImageIcon = React.forwardRef(function (
  props: RobotImageIconProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const {
    robot,
    footprint,
    iconPath,
    dispatchIconError,
    inConflict,
    colorManager,
    fleetName,
  } = props;

  // The default icon uses footprint as the radius, so we * 2 here because the width/height
  // is in a square. With the double size of the footprint, we achieved a similar
  // size to the robot default svg icon.
  const [imgIconWidth, imgIconHeigth] = useMemo(() => [footprint * 2, footprint * 2], [footprint]);
  const [robotColor, setRobotColor] = useState<string | null>(() =>
    colorManager.robotColorFromCache(fleetName, robot.name),
  );

  React.useLayoutEffect(() => {
    if (robotColor) {
      return;
    }
    (async () => {
      await colorManager
        .robotPrimaryColor(fleetName, robot.name, robot.model, iconPath)
        .then((color) => {
          if (color) setRobotColor(color);
          else {
            dispatchIconError && dispatchIconError();
          }
        })
        .catch(() => {
          dispatchIconError && dispatchIconError();
        });
    })();
  }, [robot, robotColor, colorManager, iconPath, fleetName, dispatchIconError]);

  const componentId = React.useMemo(uniqueId, []);
  const shadowId = React.useMemo(() => `RobotImageIcon-${componentId}-shadow`, [componentId]);
  const conflictShadowId = React.useMemo(() => `RobotImageIcon-${componentId}-shadow-conflict`, [
    componentId,
  ]);

  const Shadow = React.useMemo(() => makeGradientShadow('#000000'), []);
  const ShadowConflict = React.useMemo(() => makeGradientShadow(colorManager.conflictHighlight), [
    colorManager.conflictHighlight,
  ]);

  return (
    <>
      {!!iconPath && (
        <g>
          <defs>
            <Shadow id={shadowId} />
            <ShadowConflict id={conflictShadowId} />
          </defs>
          <circle
            id="shadow"
            r={footprint * 1.3}
            fill={inConflict ? `url(#${conflictShadowId})` : `url(#${shadowId})`}
          />
          <ImageIcon
            iconPath={iconPath}
            height={imgIconHeigth}
            width={imgIconWidth}
            footprint={footprint}
            dispatchIconError={dispatchIconError}
          />
        </g>
      )}
    </>
  );
});

export default RobotImageIcon;
