import React, { SVGProps, useMemo } from 'react';
import { uniqueId } from '../../util/css-utils';
import { RobotProps } from './robot';

type RobotImageIconProps = Omit<RobotProps, 'fleetName'> & {
  iconPath: string;
  dispatchIconError: React.Dispatch<
    React.SetStateAction<{
      path: string | null;
      error: boolean;
    }>
  >;
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

const RobotImageIcon = React.forwardRef(function(
  props: RobotImageIconProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const { footprint, iconPath, dispatchIconError, inConflict, colorManager } = props;

  // The default icon uses footprint as the radius, so we * 2 here because the width/height
  // is in a square. With the double size of the footprint, we achieved a similar
  // size to the robot default svg icon.
  const [imgIconWidth, imgIconHeigth] = useMemo(() => [footprint * 2, footprint * 2], [footprint]);

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
