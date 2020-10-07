import React from 'react';
import { makeStyles } from '@material-ui/core';

export interface ArrowProps {
  x: number;
  y: number;
  size?: number;
  strokeWidth?: number;
  padding?: number[];
}
/**
 * x1, y1: upper vertex of the triangle
 * x2, y2: left vertex of the triangle
 * x3, y3: right vertex of the triangle
 */
export interface BaseArrowProps {
  x1: number;
  y1: number;
  x2: number;
  y2: number;
  x3: number;
  y3: number;
  strokeWidth?: number;
  padding?: number[];
}

export const DownArrow = (props: ArrowProps) => {
  const { x, y, size = 1, strokeWidth = 0.01, padding } = props;
  let x2: number, y2: number, x3: number, y3: number;
  x2 = x - (x / 2) * size;
  y2 = y < 0 ? y + y * size : y - y * size;
  x3 = x + (x / 2) * size;
  y3 = y2;
  return (
    <Arrow
      x1={x}
      y1={y}
      x2={x2}
      y2={y2}
      x3={x3}
      y3={y3}
      strokeWidth={strokeWidth}
      padding={padding}
    />
  );
};

export const UpArrow = (props: ArrowProps) => {
  const { x, y, size = 1, strokeWidth, padding } = props;
  let x2: number, y2: number, x3: number, y3: number;
  x2 = x - (x / 2) * size;
  y2 = y < 0 ? y - y * size : y + y * size;
  x3 = x + (x / 2) * size;
  y3 = y2;
  return (
    <Arrow
      x1={x}
      y1={y}
      x2={x2}
      y2={y2}
      x3={x3}
      y3={y3}
      strokeWidth={strokeWidth}
      padding={padding}
    />
  );
};

/**
 * To represent an arrow we a triangle. To build that triangle we use a SVG polygon element.
 */
export const Arrow = (props: BaseArrowProps) => {
  const { x1, y1, x2, y2, x3, y3, strokeWidth, padding = [0, 0] } = props;
  const classes = useStyles();
  return (
    <polygon
      points={
        `${x1 + padding[0]},${y1 + padding[1]} ` +
        `${x2 + padding[0]},${y2 + padding[1]} ` +
        `${x3 + padding[0]},${y3 + padding[1]}`
      }
      strokeWidth={strokeWidth}
      className={classes.arrow}
    />
  );
};

const useStyles = makeStyles(() => ({
  arrow: {
    fill: 'black',
  },
}));
