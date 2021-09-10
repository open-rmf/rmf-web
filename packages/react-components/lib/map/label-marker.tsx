import { useTheme } from '@material-ui/core';
import React from 'react';

export interface LabelContainerProps extends React.SVGProps<SVGGElement> {
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
  children?: React.ReactNode;
}

export function LabelContainer({
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
}: LabelContainerProps): JSX.Element {
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
  extends Omit<LabelContainerProps, 'contentWidth' | 'contentHeight' | 'contentPadding'> {
  text: string;
  fontSize: number;
}

export function NameLabel({ text, fontSize, ...otherProps }: NameLabelProps): JSX.Element {
  const [contentWidth, setContentWidth] = React.useState(0);
  const [contentHeight, setContentHeight] = React.useState(0);

  return (
    <LabelContainer
      contentWidth={contentWidth}
      contentHeight={contentHeight}
      contentPadding={fontSize / 2}
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
        style={{ fontSize, userSelect: 'none' }}
        fill={otherProps.stroke}
        transform={`translate(${contentWidth / 2},${contentHeight / 2})`}
      >
        {text}
      </text>
    </LabelContainer>
  );
}
