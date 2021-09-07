import { useTheme } from '@material-ui/core';
import React from 'react';

export interface MarkerContainerProps
  extends Omit<React.SVGProps<SVGSVGElement>, 'viewBox' | 'preserveAspectRatio' | 'overflow'> {
  /**
   * Angle of the arrow pointer, in deg.
   * Default: -45
   */
  theta?: number;
  /**
   * TODO: Allow the arrow to points towards left.
   */
  // variant?: 'normal' | 'reverse';
  children?: React.ReactElement<React.SVGProps<SVGGElement>>;
}

export function MarkerContainer({
  theta = -45,
  children,
  ...otherProps
}: MarkerContainerProps): JSX.Element {
  const theme = useTheme();
  const stroke = theme.palette.text.primary;
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
  const strokeWidth = 0.003;
  const margin = 0.01;
  const [contentSize, setContentSize] = React.useState([1, 1]);
  const lineTarget = React.useMemo(() => [Math.cos(thetaRad) * 0.05, Math.sin(thetaRad) * 0.05], [
    thetaRad,
  ]);
  const width = contentSize[0] + margin * 2;
  const height = contentSize[1] + margin * 2;
  const rectStart = React.useMemo(() => {
    if (normalizedTheta >= 0 && normalizedTheta < 90) {
      return lineTarget;
    } else if (normalizedTheta >= 90 && normalizedTheta < 180) {
      return [lineTarget[0] - width, lineTarget[1]];
    } else if (normalizedTheta >= -180 && normalizedTheta < -90) {
      return [lineTarget[0] - width, lineTarget[1] - height];
    } else {
      return [lineTarget[0], lineTarget[1] - height];
    }
  }, [normalizedTheta, width, height, lineTarget]);

  return (
    <svg {...otherProps} viewBox={`0 0 1 1`} preserveAspectRatio="xMinYMin" overflow="visible">
      <line
        x1={0}
        y1={0}
        x2={lineTarget[0]}
        y2={lineTarget[1]}
        stroke={stroke}
        strokeWidth={strokeWidth}
        strokeLinecap="square"
      />
      <rect
        x={rectStart[0]}
        y={rectStart[1]}
        width={width}
        height={height}
        rx={0.01}
        ry={0.01}
        fill="none"
        stroke={stroke}
        strokeWidth={strokeWidth}
      />
      <g
        ref={(current) => {
          if (!current) return;
          const bbox = current.getBBox();
          setContentSize((prev) => {
            if (prev[0] === bbox.width && prev[1] === bbox.height) return prev;
            return [bbox.width, bbox.height];
          });
        }}
        transform={`translate(${rectStart[0] + width / 2},${rectStart[1] + height / 2})`}
      >
        {children}
      </g>
    </svg>
  );
}

export interface NameLabelProps extends MarkerContainerProps {
  text: string;
}

export function NameLabel({ text, ...otherProps }: NameLabelProps): JSX.Element {
  return (
    <MarkerContainer {...otherProps}>
      <text
        x={0}
        y={0}
        dominantBaseline="middle"
        textAnchor="middle"
        style={{ fontSize: 0.05, userSelect: 'none' }}
      >
        {text}
      </text>
    </MarkerContainer>
  );
}
