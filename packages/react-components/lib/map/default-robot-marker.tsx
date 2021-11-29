import { useTheme } from '@mui/material';
import React from 'react';
import { ColorContext } from '../color-manager';
import { uniqueId } from '../utils';
import type { RobotMarkerProps } from './robot-marker';

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

export const DefaultMarker = ({
  cx,
  cy,
  r,
  color,
  inConflict = false,
}: RobotMarkerProps): JSX.Element => {
  const colorManager = React.useContext(ColorContext);
  const theme = useTheme();

  const componentId = React.useMemo(uniqueId, []);
  const shadowId = React.useMemo(() => `RobotDefaultIcon-${componentId}-shadow`, [componentId]);
  const conflictShadowId = React.useMemo(
    () => `RobotDefaultIcon-${componentId}-shadow-conflict`,
    [componentId],
  );

  const Shadow = React.useMemo(() => makeGradientShadow('#000000'), []);
  const ShadowConflict = React.useMemo(
    () => makeGradientShadow(colorManager.conflictHighlight),
    [colorManager.conflictHighlight],
  );

  return (
    <g>
      <defs>
        <Shadow id={shadowId} />
        <ShadowConflict id={conflictShadowId} />
      </defs>
      <circle
        r={r * 1.3}
        cx={cx}
        cy={cy}
        fill={inConflict ? `url(#${conflictShadowId})` : `url(#${shadowId})`}
      />
      <circle r={r} cx={cx} cy={cy} fill={color} />
      <line
        x1={cx}
        y1={cy}
        x2={cx + r}
        y2={cy}
        stroke={theme.palette.common.black}
        strokeWidth="0.05"
      />
    </g>
  );
};

export default DefaultMarker;
