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
      <stop offset="0%" stopColor={`${color}80`} />
      <stop offset="70%" stopColor={`${color}40`} />
      <stop offset="90%" stopColor={`${color}10`} />
      <stop offset="100%" stopColor={`${color}00`} />
    </radialGradient>
  );
}

export interface ImageMarkerProps extends Omit<RobotMarkerProps, 'color'> {
  iconPath: string;
  onError?: React.EventHandler<React.SyntheticEvent<SVGImageElement, Event>>;
}

// TODO: Support rectangle markers?
/**
 * Image should be 1x1 aspect ratio.
 */
export const ImageMarker = ({
  cx,
  cy,
  r,
  iconPath,
  inConflict = false,
  onError,
}: ImageMarkerProps): JSX.Element | null => {
  const colorManager = React.useContext(ColorContext);

  const componentId = React.useMemo(uniqueId, []);
  const shadowId = React.useMemo(() => `RobotImageIcon-${componentId}-shadow`, [componentId]);
  const conflictShadowId = React.useMemo(() => `RobotImageIcon-${componentId}-shadow-conflict`, [
    componentId,
  ]);

  const Shadow = React.useMemo(() => makeGradientShadow('#000000'), []);
  const ShadowConflict = React.useMemo(() => makeGradientShadow(colorManager.conflictHighlight), [
    colorManager.conflictHighlight,
  ]);

  return iconPath ? (
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
      <image href={iconPath} width={r * 2} height={r * 2} x={cx - r} y={cy - r} onError={onError} />
    </g>
  ) : null;
};

export default ImageMarker;
