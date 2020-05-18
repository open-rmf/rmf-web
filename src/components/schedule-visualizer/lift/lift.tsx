import { makeStyles } from '@material-ui/core';
import React from 'react';

interface LiftProps {
  lift: {
    /**
     * Position x
     * @required
     */
    x: number;
    y: number;
    width: number;
    height: number;
    rx?: number;
    ry?: number;
    transform?: string;
    style?: any;
  };
  liftMotionText: {
    x: number;
    y: number;
    text: string | undefined;
    style?: any;
    fontSize?: string;
  };
  liftModeText: {
    x: number;
    y: number;
    text: string | undefined;
    style?: any;
    fontSize?: string;
  };

  onClick?: any; //(e: React.MouseEvent<SVGGElement>, lift: RomiCoreLift): void;
}

const Lift = React.forwardRef(function(
  props: LiftProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const { lift, onClick, liftMotionText, liftModeText } = props;
  const { width, height, x, y, rx, ry, transform, style } = lift;
  const {
    x: motionX,
    y: motionY,
    text: motionText,
    style: motionStyle,
    fontSize: motionFontSize,
  } = liftMotionText;
  const {
    x: modeX,
    y: modeY,
    text: modeText,
    style: modeStyle,
    fontSize: modeFontSize,
  } = liftModeText;

  const classes = useStyles();
  return (
    <>
      <g ref={ref} onClick={event => onClick(event)}>
        <rect
          className={`${classes.liftMarker} ${classes.lift} ${style || null}`}
          width={width}
          height={height}
          x={x}
          y={y}
          rx={rx || 0}
          ry={ry || 0}
          transform={transform}
        />
        {motionText && (
          <text
            id="liftMotion"
            className={motionStyle || classes.liftText}
            x={motionX}
            y={motionY}
            fontSize={motionFontSize || '1rem'}
          >
            {motionText}
          </text>
        )}
        {liftModeText && (
          <text
            id="liftMode"
            className={modeStyle || classes.liftText}
            x={modeX}
            y={modeY}
            fontSize={modeFontSize || '1rem'}
          >
            {modeText}
          </text>
        )}
      </g>
    </>
  );
});

export default Lift;

const useStyles = makeStyles(() => ({
  liftMarker: {
    cursor: 'pointer',
    pointerEvents: 'auto',
  },
  lift: {
    strokeWidth: '0.2',
  },
  liftText: {
    dominantBaseline: 'central',
    textAnchor: 'middle',
    fontWeight: 'bold',
  },
}));
