import { useTheme } from '@material-ui/core';
import React from 'react';

export interface MarkerContainerProps extends React.SVGProps<SVGGElement> {
  anchorX: number;
  anchorY: number;
  arrowLength: number;
  contentWidth: number;
  contentHeight: number;
  contentPadding?: number;
  radius?: number;
  /**
   * Angle of the arrow pointer, in deg.
   * Default: -30
   */
  theta?: number;
  /**
   * TODO: Allow the arrow to points towards left.
   */
  // variant?: 'normal' | 'reverse';
  children?: React.ReactNode;
}

export function MarkerContainer({
  anchorX,
  anchorY,
  arrowLength,
  contentWidth,
  contentHeight,
  contentPadding = 0,
  radius = 0,
  theta = -30,
  children,
  ...otherProps
}: MarkerContainerProps): JSX.Element {
  const theme = useTheme();
  const contentWidthOuter = contentWidth + contentPadding * 2;
  const contentHeightOuter = contentHeight + contentPadding * 2;
  const normalizedTheta = React.useMemo(() => {
    let cur = theta;
    while (cur > 180 || cur < -180) {
      if (cur > 180) {
        cur -= 180;
      } else {
        cur += 180;
      }
    }
    return cur;
  }, [theta]);
  const thetaRad = (normalizedTheta * Math.PI) / 180;
  const lineTarget = React.useMemo(
    () => [Math.cos(thetaRad) * arrowLength, Math.sin(thetaRad) * arrowLength],
    [thetaRad, arrowLength],
  );
  const rectStart = React.useMemo(() => {
    if (normalizedTheta >= 0 && normalizedTheta < 90) {
      return [lineTarget[0] - radius, lineTarget[1]];
    } else if (normalizedTheta >= 90 && normalizedTheta < 180) {
      return [lineTarget[0] - contentWidthOuter + radius, lineTarget[1]];
    } else if (normalizedTheta >= -180 && normalizedTheta < -90) {
      return [lineTarget[0] - contentWidthOuter + radius, lineTarget[1] - contentHeightOuter];
    } else {
      return [lineTarget[0] - radius, lineTarget[1] - contentHeightOuter];
    }
  }, [normalizedTheta, contentWidthOuter, contentHeightOuter, lineTarget, radius]);

  return (
    <g transform={`translate(${anchorX},${anchorY})`}>
      <g {...otherProps}>
        <line x1={0} y1={0} x2={lineTarget[0]} y2={lineTarget[1]} mask="url(#asdasd)" />
        <rect
          x={rectStart[0]}
          y={rectStart[1]}
          width={contentWidthOuter}
          height={contentHeightOuter}
          rx={radius}
          ry={radius}
          fill={theme.palette.background.paper}
        />
        <g
          transform={`translate(${rectStart[0] + contentPadding},${rectStart[1] + contentPadding})`}
        >
          {children}
        </g>
      </g>
    </g>
  );
}

export interface NameLabelProps
  extends Omit<MarkerContainerProps, 'arrowLength' | 'contentWidth' | 'contentHeight'> {
  text: string;
}

export function NameLabel({ text, ...otherProps }: NameLabelProps): JSX.Element {
  const theme = useTheme();
  const stroke = theme.palette.primary.main;
  const [contentWidth, setContentWidth] = React.useState(0.1);
  const [contentHeight, setContentHeight] = React.useState(0.1);

  return (
    <MarkerContainer
      arrowLength={0.05}
      contentWidth={contentWidth}
      contentHeight={contentHeight}
      contentPadding={0.005}
      radius={0.01}
      stroke={stroke}
      strokeWidth={0.003}
      {...otherProps}
    >
      <text
        ref={(current) => {
          if (!current) return;
          const bbox = current.getBBox();
          setContentWidth(bbox.width);
          setContentHeight(bbox.height);
        }}
        x={0}
        y={0}
        strokeWidth={0}
        dominantBaseline="middle"
        textAnchor="middle"
        style={{ fontSize: 0.05, userSelect: 'none' }}
        fill={stroke}
        transform={`translate(${contentWidth / 2},${contentHeight / 2})`}
      >
        {text}
      </text>
    </MarkerContainer>
  );
}
