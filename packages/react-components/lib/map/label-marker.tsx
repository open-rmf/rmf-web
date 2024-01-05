import { styled, useTheme } from '@mui/material';
import clsx from 'clsx';
import React from 'react';
import { BBox, EntityManagerContext } from './entity-manager';
import { useAutoScale } from './hooks';

const DefaultRepositionThreshold = 200;

export interface LabelLocation {
  anchorX1: number;
  anchorY1: number;
  anchorX2: number;
  anchorY2: number;
  borderBBox: BBox;
}

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
  /**
   * Maximum distance to reposition the label.
   */
  repositionThreshold?: number;
  children?: React.ReactNode;
}

export function LabelContainer(props: LabelContainerProps): JSX.Element | null {
  const theme = useTheme();
  const {
    sourceX,
    sourceY,
    sourceRadius,
    contentWidth,
    contentHeight,
    // theme.spacing default value = 8
    contentPadding = 8,
    // theme.shape.borderRadius default = 4
    contentBorderRadius = 2,
    arrowLength: preferredArrowLength = 8,
    angle = -30,
    repositionThreshold = DefaultRepositionThreshold,
    children,
    ...otherProps
  } = props;
  const entityManager = React.useContext(EntityManagerContext);
  const contentWidthOuter = contentWidth + contentPadding * 2;
  const contentHeightOuter = contentHeight + contentPadding * 2;

  const preferredLocation = React.useMemo<LabelLocation>(() => {
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
    const theta = (normalizedAngle * Math.PI) / 180;
    const anchorX1 = sourceX + Math.cos(theta) * sourceRadius;
    const anchorY1 = sourceY + Math.sin(theta) * sourceRadius;
    const anchorX2 = anchorX1 + Math.cos(theta) * preferredArrowLength;
    const anchorY2 = anchorY1 + Math.sin(theta) * preferredArrowLength;
    const rectStart = (() => {
      if (theta >= 0 && theta < Math.PI / 2) {
        return [anchorX2 - contentBorderRadius, anchorY2];
      } else if (theta >= Math.PI / 2 && theta < Math.PI) {
        return [anchorX2 - contentWidthOuter + contentBorderRadius, anchorY2];
      } else if (theta >= -Math.PI && theta < -Math.PI / 2) {
        return [anchorX2 - contentWidthOuter + contentBorderRadius, anchorY2 - contentHeightOuter];
      } else {
        return [anchorX2 - contentBorderRadius, anchorY2 - contentHeightOuter];
      }
    })();
    return {
      anchorX1,
      anchorX2,
      anchorY1,
      anchorY2,
      borderBBox: {
        minX: rectStart[0],
        minY: rectStart[1],
        maxX: rectStart[0] + contentWidthOuter,
        maxY: rectStart[1] + contentHeightOuter,
      },
    };
  }, [
    angle,
    contentBorderRadius,
    contentWidthOuter,
    contentHeightOuter,
    preferredArrowLength,
    sourceRadius,
    sourceX,
    sourceY,
  ]);

  const [labelLocation, setLabelLocation] = React.useState<LabelLocation | null>(null);
  React.useLayoutEffect(() => {
    const nonColliding = entityManager.getNonColliding(preferredLocation.borderBBox, {
      distLimit: repositionThreshold,
    });
    if (!nonColliding) return;

    const entity = entityManager.add({ bbox: nonColliding });
    if (nonColliding !== preferredLocation.borderBBox) {
      const width = nonColliding.maxX - nonColliding.minX;
      const height = nonColliding.maxY - nonColliding.minY;
      const centerX = nonColliding.minX + width / 2;
      const centerY = nonColliding.minY + height / 2;
      const theta = Math.atan2(centerY - sourceY, centerX - sourceX);
      const anchorX1 = sourceX + Math.cos(theta) * sourceRadius;
      const anchorY1 = sourceY + Math.sin(theta) * sourceRadius;
      const anchorX2 = (() => {
        if (theta >= -Math.PI / 2 && theta < Math.PI / 2) {
          return nonColliding.minX + contentBorderRadius;
        } else {
          return nonColliding.maxX - contentBorderRadius;
        }
      })();
      const anchorY2 = (() => {
        if (theta >= 0 && theta < Math.PI) {
          return nonColliding.minY;
        } else {
          return nonColliding.maxY;
        }
      })();

      setLabelLocation({
        anchorX1,
        anchorY1,
        anchorX2,
        anchorY2,
        borderBBox: nonColliding,
      });
    } else {
      setLabelLocation(preferredLocation);
    }

    return () => {
      entityManager.remove(entity);
    };
  }, [
    entityManager,
    preferredLocation,
    contentBorderRadius,
    sourceRadius,
    sourceX,
    sourceY,
    repositionThreshold,
  ]);

  return (
    labelLocation && (
      <g {...otherProps}>
        <line
          x1={labelLocation.anchorX1}
          y1={labelLocation.anchorY1}
          x2={labelLocation.anchorX2}
          y2={labelLocation.anchorY2}
        />
        <rect
          x={labelLocation.borderBBox.minX}
          y={labelLocation.borderBBox.minY}
          width={labelLocation.borderBBox.maxX - labelLocation.borderBBox.minX}
          height={labelLocation.borderBBox.maxY - labelLocation.borderBBox.minY}
          rx={contentBorderRadius}
          ry={contentBorderRadius}
          fill={theme.palette.background.paper}
        />
        <g
          transform={`translate(${labelLocation.borderBBox.minX + contentPadding} ${
            labelLocation.borderBBox.minY + contentPadding
          })`}
        >
          {children}
        </g>
      </g>
    )
  );
}

const classes = {
  container: 'name-label-container',
};

const StyledLabelContainer = styled((props: LabelContainerProps) => <LabelContainer {...props} />)(
  ({ theme }) => ({
    [`&.${classes.container}`]: {
      fontSize: theme.typography.fontSize,
      fontFamily: theme.typography.fontFamily,
      userSelect: 'none',
    },
  }),
);

const Text: React.FC<React.PropsWithRef<React.SVGProps<SVGTextElement>>> = React.forwardRef(
  ({ children, ...otherProps }: React.SVGProps<SVGTextElement>, ref: React.Ref<SVGTextElement>) => {
    return (
      <text
        ref={ref}
        x={0}
        y={0}
        strokeWidth={0}
        dominantBaseline="middle"
        textAnchor="middle"
        {...otherProps}
      >
        {children}
      </text>
    );
  },
);

export interface NameLabelProps
  extends Omit<LabelContainerProps, 'contentWidth' | 'contentHeight'> {
  text: string;
}

export function NameLabel(props: NameLabelProps): JSX.Element {
  const theme = useTheme();
  const { text, contentPadding = 4, className, ...otherProps } = props;
  const [contentWidth, setContentWidth] = React.useState(0);
  const [contentHeight, setContentHeight] = React.useState(0);
  const [show, setShow] = React.useState(false);
  const textRef = React.useRef<SVGTextElement>(null);

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
        setContentWidth(bbox.width + 16);
        setContentHeight(bbox.height);
        setShow(true);
      }
    };
    updateContentSize();
  }, [theme]);

  return (
    <>
      {/* Dummy to compute text length */}
      {!show && (
        <Text ref={textRef} style={{ visibility: 'hidden' }}>
          {text}
        </Text>
      )}
      {show && (
        <StyledLabelContainer
          contentWidth={contentWidth}
          contentHeight={contentHeight}
          contentPadding={contentPadding}
          className={clsx(classes.container, className)}
          stroke={theme.palette.info.main}
          strokeWidth={1}
          {...otherProps}
        >
          <Text
            fill={theme.palette.info.main}
            transform={`translate(${contentWidth / 2},${contentHeight / 2})`}
          >
            {text}
          </Text>
        </StyledLabelContainer>
      )}
    </>
  );
}

export type ScalableLabelProps = Pick<
  LabelContainerProps,
  'sourceX' | 'sourceY' | 'sourceRadius' | 'arrowLength' | 'repositionThreshold' | 'transform'
>;

export function withAutoScaling<PropsType extends ScalableLabelProps>(
  LabelComponent: React.ComponentType<PropsType>,
): React.ComponentType<PropsType> {
  return ({
    sourceX,
    sourceY,
    sourceRadius,
    arrowLength,
    repositionThreshold = 5,
    transform,
    ...otherProps
  }: PropsType): JSX.Element => {
    const scale = useAutoScale(1.2, Infinity);
    // getBBox returns the bbox BEFORE transform. Not pre-scaling the source x, y will cause the
    // collision detection to think the bbox is different size than what it actually is on screen.
    return (
      <LabelComponent
        {...(otherProps as unknown as PropsType)}
        sourceX={sourceX / scale}
        sourceY={sourceY / scale}
        sourceRadius={sourceRadius / scale}
        arrowLength={arrowLength ? arrowLength / scale : undefined}
        repositionThreshold={repositionThreshold / scale}
        transform={clsx(transform, `scale(${scale})`)}
      />
    );
  };
}

export const ScaledNameLabel = withAutoScaling(NameLabel);
