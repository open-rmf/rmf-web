import { useTheme } from '@material-ui/core';
import React, { SVGProps, useState } from 'react';
import { uniqueId } from '../../util/css-utils';
import { RobotProps } from './robot';

type RobotDefaultIconProps = Omit<RobotProps, 'fleetName'>;

/**
 *
 * @param color MUST be in hex notation without alpha channel. e.g. #123456
 */
function makeGradientShadow(
  color: string,
): React.FunctionComponent<SVGProps<SVGRadialGradientElement>> {
  return (props: SVGProps<SVGRadialGradientElement>) => (
    <radialGradient {...props}>
      <stop offset="70%" stopColor={`${color}ff`} />
      <stop offset="75%" stopColor={`${color}80`} />
      <stop offset="80%" stopColor={`${color}60`} />
      <stop offset="85%" stopColor={`${color}30`} />
      <stop offset="90%" stopColor={`${color}18`} />
      <stop offset="95%" stopColor={`${color}08`} />
      <stop offset="100%" stopColor={`${color}00`} />
    </radialGradient>
  );
}

const RobotDefaultIcon = React.forwardRef(function(
  props: RobotDefaultIconProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const { robot, footprint, colorManager, inConflict } = props;
  const [robotColor, setRobotColor] = useState<string | null>(() =>
    colorManager.robotColorFromCache(robot.name, robot.model),
  );
  const theme = useTheme();

  const componentId = React.useMemo(uniqueId, []);
  const shadowId = React.useMemo(() => `RobotDefaultIcon-${componentId}-shadow`, [componentId]);
  const conflictShadowId = React.useMemo(() => `RobotDefaultIcon-${componentId}-shadow-conflict`, [
    componentId,
  ]);

  const Shadow = React.useMemo(() => makeGradientShadow('#000000'), []);
  const ShadowConflict = React.useMemo(() => makeGradientShadow(colorManager.conflictHighlight), [
    colorManager.conflictHighlight,
  ]);

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
            <Shadow id={shadowId} />
            <ShadowConflict id={conflictShadowId} />
          </defs>
          <circle
            id="shadow"
            r={footprint * 1.3}
            fill={inConflict ? `url(#${conflictShadowId})` : `url(#${shadowId})`}
          />
          <circle r={footprint} fill={robotColor} />
          <line x2={footprint} stroke={theme.palette.common.black} strokeWidth="0.05" />
        </g>
      )}
    </>
  );
});

export default RobotDefaultIcon;
