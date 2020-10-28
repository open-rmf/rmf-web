import { useTheme } from '@material-ui/core';
import React from 'react';
import { ColorContext } from '../color-manager';
import { uniqueId } from '../css-utils';
import { RobotMarkerProps } from './robot-marker';

/**
 *
 * @param color MUST be in hex notation without alpha channel. e.g. #123456
 */
function makeGradientShadow(
  color: string,
): React.FunctionComponent<React.SVGProps<SVGRadialGradientElement>> {
  return (props: React.SVGProps<SVGRadialGradientElement>) => (
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

export const DefaultMarker = (props: RobotMarkerProps): JSX.Element | null => {
  const { robot, footprint, fleetName, variant = 'normal' } = props;
  const colorManager = React.useContext(ColorContext);
  const [robotColor, setRobotColor] = React.useState<string | null>(() =>
    colorManager.robotColorFromCache(fleetName, robot.name),
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
      setRobotColor(await colorManager.robotPrimaryColor(fleetName, robot.name, robot.model));
      await colorManager.robotPrimaryColor(fleetName, robot.name, robot.model);
    })();
  }, [robot, robotColor, colorManager, fleetName]);

  return robotColor ? (
    <g>
      <defs>
        <Shadow id={shadowId} />
        <ShadowConflict id={conflictShadowId} />
      </defs>
      <circle
        id="shadow"
        r={footprint * 1.3}
        fill={variant === 'inConflict' ? `url(#${conflictShadowId})` : `url(#${shadowId})`}
      />
      <circle r={footprint} fill={robotColor} />
      <line x2={footprint} stroke={theme.palette.common.black} strokeWidth="0.05" />
    </g>
  ) : null;
};

export default DefaultMarker;
