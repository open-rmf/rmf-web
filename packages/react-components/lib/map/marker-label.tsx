import { useTheme } from '@material-ui/core';
import React from 'react';

export interface MarkerContainerProps extends Omit<React.SVGProps<SVGSVGElement>, 'overflow'> {
  arrowLength: number;
  contentWidth: number;
  contentHeight: number;
  contentPadding?: number;
  /**
   * Angle of the arrow pointer, in deg.
   * Default: -45
   */
  theta?: number;
  /**
   * TODO: Allow the arrow to points towards left.
   */
  // variant?: 'normal' | 'reverse';
  children?: React.ReactNode;
}

export function MarkerContainer({
  arrowLength,
  contentWidth,
  contentHeight,
  contentPadding = 0,
  theta = -45,
  children,
  ...otherProps
}: MarkerContainerProps): JSX.Element {
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
      return lineTarget;
    } else if (normalizedTheta >= 90 && normalizedTheta < 180) {
      return [lineTarget[0] - contentWidthOuter, lineTarget[1]];
    } else if (normalizedTheta >= -180 && normalizedTheta < -90) {
      return [lineTarget[0] - contentWidthOuter, lineTarget[1] - contentHeightOuter];
    } else {
      return [lineTarget[0], lineTarget[1] - contentHeightOuter];
    }
  }, [normalizedTheta, contentWidthOuter, contentHeightOuter, lineTarget]);

  return (
    <svg {...otherProps} overflow="visible">
      <line x1={0} y1={0} x2={lineTarget[0]} y2={lineTarget[1]} strokeLinecap="square" />
      <rect
        x={rectStart[0]}
        y={rectStart[1]}
        width={contentWidthOuter}
        height={contentHeightOuter}
        rx={0.01}
        ry={0.01}
        fill="none"
      />
      <g transform={`translate(${rectStart[0] + contentPadding},${rectStart[1] + contentPadding})`}>
        {children}
      </g>
    </svg>
  );
}

export interface NameLabelProps
  extends Omit<
    MarkerContainerProps,
    'arrowLength' | 'contentWidth' | 'contentHeight' | 'contentPadding'
  > {
  text: string;
}

export function NameLabel({ text, ...otherProps }: NameLabelProps): JSX.Element {
  const theme = useTheme();
  const stroke = theme.palette.text.primary;
  const [contentWidth, setContentWidth] = React.useState(0.1);
  const [contentHeight, setContentHeight] = React.useState(0.1);

  return (
    <svg viewBox="0 0 1 1" preserveAspectRatio="xMinYMin" overflow="visible">
      <MarkerContainer
        {...otherProps}
        arrowLength={0.05}
        contentWidth={contentWidth}
        contentHeight={contentHeight}
        contentPadding={0.005}
        stroke={stroke}
        strokeWidth={0.003}
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
          transform={`translate(${contentWidth / 2},${contentHeight / 2})`}
        >
          {text}
        </text>
      </MarkerContainer>
    </svg>
  );
}
