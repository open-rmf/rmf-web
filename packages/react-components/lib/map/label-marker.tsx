import { makeStyles, useTheme } from '@material-ui/core';
import clsx from 'clsx';
import Debug from 'debug';
import RBush, { BBox } from 'rbush';
import React from 'react';
import { useAutoScale } from './hooks';

const debug = Debug('Map:LabelMarker');

type LabelLocation = BBox;

export class LabelManager {
  addLabel(preferredLocation: LabelLocation): LabelLocation {
    const labelLocation = this._getLocation(preferredLocation);
    this._labels.insert(labelLocation);
    return labelLocation;
  }

  removeLabel(label: LabelLocation): void {
    this._labels.remove(label);
  }

  private _labels = new RBush<LabelLocation>();

  private _getLocation(preferredLocation: LabelLocation): LabelLocation {
    return preferredLocation;
  }
}

export const LabelManagerContext = React.createContext(new LabelManager());

export interface LabelContainerProps extends React.SVGProps<SVGGElement> {
  sourceX: number;
  sourceY: number;
  sourceRadius: number;
  contentWidth: number;
  contentHeight: number;
  contentPadding?: number;
  contentBorderRadius?: number;
  arrowLength?: number;
  /**
   * Preferred angle of the arrow pointer, in deg.
   * Default: -30
   */
  angle?: number;
  children?: React.ReactNode;
}

export function LabelContainer(props: LabelContainerProps): JSX.Element {
  const theme = useTheme();
  const {
    sourceX,
    sourceY,
    sourceRadius,
    contentWidth,
    contentHeight,
    contentPadding = theme.spacing(1),
    contentBorderRadius = theme.shape.borderRadius / 2,
    arrowLength: preferredArrowLength = theme.spacing(1),
    angle = -30,
    children,
    ...otherProps
  } = props;
  const labelManager = React.useContext(LabelManagerContext);
  const contentWidthOuter = contentWidth + contentPadding * 2;
  const contentHeightOuter = contentHeight + contentPadding * 2;
  const preferredTheta = React.useMemo(() => {
    const normalizedAngle = (() => {
      let cur = angle;
      while (cur > 180 || cur < -180) {
        if (cur > 180) {
          cur -= 180;
        } else {
          cur += 180;
        }
      }
      return cur;
    })();
    return (normalizedAngle * Math.PI) / 180;
  }, [angle]);
  const preferredAnchorX1 = React.useMemo(() => sourceX + Math.cos(preferredTheta) * sourceRadius, [
    sourceX,
    sourceRadius,
    preferredTheta,
  ]);
  const preferredAnchorY1 = React.useMemo(() => sourceY + Math.sin(preferredTheta) * sourceRadius, [
    sourceY,
    sourceRadius,
    preferredTheta,
  ]);
  const preferredAnchorX2 = React.useMemo(
    () => preferredAnchorX1 + Math.cos(preferredTheta) * preferredArrowLength,
    [preferredAnchorX1, preferredTheta, preferredArrowLength],
  );
  const preferredAnchorY2 = React.useMemo(
    () => preferredAnchorY1 + Math.sin(preferredTheta) * preferredArrowLength,
    [preferredAnchorY1, preferredTheta, preferredArrowLength],
  );

  const preferredLocation = React.useMemo(() => {
    const rectStart = (() => {
      if (preferredTheta >= 0 && preferredTheta < Math.PI / 2) {
        return [preferredAnchorX2 - contentBorderRadius, preferredAnchorY2];
      } else if (preferredTheta >= Math.PI / 2 && preferredTheta < Math.PI) {
        return [preferredAnchorX2 - contentWidthOuter + contentBorderRadius, preferredAnchorY2];
      } else if (preferredTheta >= -Math.PI && preferredTheta < -Math.PI / 2) {
        return [
          preferredAnchorX2 - contentWidthOuter + contentBorderRadius,
          preferredAnchorY2 - contentHeightOuter,
        ];
      } else {
        return [preferredAnchorX2 - contentBorderRadius, preferredAnchorY2 - contentHeightOuter];
      }
    })();

    return {
      minX: rectStart[0],
      minY: rectStart[1],
      maxX: rectStart[0] + contentWidthOuter,
      maxY: rectStart[1] + contentHeightOuter,
    };
  }, [
    preferredTheta,
    preferredAnchorX2,
    preferredAnchorY2,
    contentWidthOuter,
    contentHeightOuter,
    contentBorderRadius,
  ]);
  const [labelLocation, setLabelLocation] = React.useState(preferredLocation);
  React.useEffect(() => {
    const newLocation = labelManager.addLabel(preferredLocation);
    setLabelLocation(newLocation);
    return () => {
      labelManager.removeLabel(newLocation);
    };
  }, [labelManager, preferredLocation]);

  const theta = React.useMemo(() => {
    const width = labelLocation.maxX - labelLocation.minX;
    const height = labelLocation.maxY - labelLocation.minY;
    const labelCenterX = labelLocation.minX + width / 2;
    const labelCenterY = labelLocation.minY + height / 2;
    return Math.atan2(labelCenterY - sourceY, labelCenterX - sourceX);
  }, [labelLocation, sourceX, sourceY]);
  const anchorX1 = React.useMemo(() => sourceX + Math.cos(theta) * sourceRadius, [
    sourceX,
    sourceRadius,
    theta,
  ]);
  const anchorY1 = React.useMemo(() => sourceY + Math.sin(theta) * sourceRadius, [
    sourceY,
    sourceRadius,
    theta,
  ]);
  const anchorX2 = React.useMemo(() => {
    if (theta >= -Math.PI / 2 && theta < Math.PI / 2) {
      return labelLocation.minX + contentBorderRadius;
    } else {
      return labelLocation.maxX - contentBorderRadius;
    }
  }, [labelLocation, theta, contentBorderRadius]);
  const anchorY2 = React.useMemo(() => {
    if (theta >= 0 && theta < Math.PI) {
      return labelLocation.minY;
    } else {
      return labelLocation.maxY;
    }
  }, [labelLocation, theta]);

  return (
    <g>
      <g id="testtest" {...otherProps}>
        <line x1={anchorX1} y1={anchorY1} x2={anchorX2} y2={anchorY2} />
        <rect
          x={labelLocation.minX}
          y={labelLocation.minY}
          width={labelLocation.maxX - labelLocation.minX}
          height={labelLocation.maxY - labelLocation.minY}
          rx={contentBorderRadius}
          ry={contentBorderRadius}
          fill={theme.palette.background.paper}
        />
        <g
          transform={`translate(${labelLocation.minX + contentPadding} ${
            labelLocation.minY + contentPadding
          })`}
        >
          {children}
        </g>
      </g>
    </g>
  );
}

const useNameLabelStyles = makeStyles((theme) => ({
  container: {
    fontSize: theme.typography.fontSize,
    fontFamily: theme.typography.fontFamily,
    userSelect: 'none',
  },
  hide: {
    visibility: 'hidden',
  },
}));

export interface NameLabelProps
  extends Omit<LabelContainerProps, 'contentWidth' | 'contentHeight'> {
  text: string;
}

export function NameLabel(props: NameLabelProps): JSX.Element {
  const theme = useTheme();
  const { text, contentPadding = theme.spacing(0.5), className, ...otherProps } = props;
  const classes = useNameLabelStyles();
  const [contentWidth, setContentWidth] = React.useState(0);
  const [contentHeight, setContentHeight] = React.useState(0);
  const [show, setShow] = React.useState(false);
  const textRef = React.useRef<SVGTextElement>(null);

  const updateContentSize = React.useCallback((bbox: DOMRect) => {
    if (bbox.width !== 0) {
      setContentWidth(bbox.width);
      setContentHeight(bbox.height);
      setShow(true);
    } else {
      debug('bbox not available, trying again in next frame');
      requestAnimationFrame(() => {
        updateContentSize(bbox);
      });
    }
  }, []);

  React.useEffect(() => {
    const updateContentSize = () => {
      if (!textRef.current) return;
      const bbox = textRef.current.getBBox();
      // sometimes the element is not rendered yet, and the bbox is not available.
      if (bbox.width === 0) {
        requestAnimationFrame(() => {
          updateContentSize();
        });
      } else {
        setContentWidth(bbox.width);
        setContentHeight(bbox.height);
        setShow(true);
      }
    };
    updateContentSize();
  }, []);

  return (
    <LabelContainer
      contentWidth={contentWidth}
      contentHeight={contentHeight}
      contentPadding={contentPadding}
      className={clsx(classes.container, !show && classes.hide, className)}
      stroke={theme.palette.primary.main}
      strokeWidth={1}
      {...otherProps}
    >
      <text
        ref={textRef}
        x={0}
        y={0}
        strokeWidth={0}
        dominantBaseline="middle"
        textAnchor="middle"
        fill={theme.palette.primary.main}
        transform={`translate(${contentWidth / 2},${contentHeight / 2})`}
      >
        {text}
      </text>
    </LabelContainer>
  );
}

export type ScalableLabelProps = Pick<
  LabelContainerProps,
  'sourceX' | 'sourceY' | 'sourceRadius' | 'transform'
>;

export function withAutoScaling<PropsType extends ScalableLabelProps>(
  LabelComponent: React.ComponentType<PropsType>,
) {
  return ({ sourceX, sourceY, sourceRadius, transform, ...otherProps }: PropsType): JSX.Element => {
    const scale = useAutoScale(1, Infinity);
    return (
      <LabelComponent
        {...((otherProps as unknown) as PropsType)}
        sourceX={0}
        sourceY={0}
        sourceRadius={sourceRadius / scale}
        transform={clsx(transform, `translate(${sourceX} ${sourceY}) scale(${scale})`)}
      />
    );
  };
}

export const ScaledNameLabel = withAutoScaling(NameLabel);
