import React from 'react';
import { ColorContext } from '../color-manager';
import { uniqueId } from '../utils';

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

export interface ImageMarkerProps {
  iconPath: string;
  inConflict?: boolean;
  onError?: React.EventHandler<React.SyntheticEvent<SVGImageElement, Event>>;
}

export const ImageMarker = ({
  iconPath,
  inConflict = false,
  onError,
}: ImageMarkerProps): JSX.Element | null => {
  const [imgIconWidth, imgIconHeight] = React.useMemo(() => [2, 2], []);
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
      <circle r={1.3} fill={inConflict ? `url(#${conflictShadowId})` : `url(#${shadowId})`} />
      <image
        href={iconPath}
        width={imgIconWidth}
        height={imgIconHeight}
        x={-1}
        y={-1}
        onError={onError}
      />
    </g>
  ) : null;
};

export default ImageMarker;
