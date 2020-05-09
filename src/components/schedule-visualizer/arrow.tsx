import React from 'react';
import { makeStyles } from '@material-ui/core';

export interface ArrowProps {
  x: number;
  y: number;
  size?: number;
  strokeWidth?: number;
}

export const UpArrow = (props: ArrowProps) => {
  const { x, y, size = 1, strokeWidth } = props;
  const classes = useStyles();
  let x2: number, y2: number, x3: number, y3: number;

  x2 = x + (x / 2) * size;
  y2 = Math.abs(y + y * size);
  x3 = x + x * size;
  y3 = y;
  console.log(`${x},${y} ${x2},${y2} ${x3},${y3} `);
  return <polygon points={`${x},${y} ${x2},${y2} ${x3},${y3} `} fill="black" />;
};

export const DownArrow = (props: ArrowProps) => {
  const { x, y, size = 1, strokeWidth } = props;
  const classes = useStyles();
  let x2: number, y2: number, x3: number, y3: number;

  x2 = x + (x / 2) * size;
  y2 = Math.abs(y + y * size);
  x3 = x + x * size;
  y3 = y;
  console.log(`${x},${y} ${x2},${y2} ${x3},${y3} `);
  return <polygon points={`${x},${y} ${x2},${y2} ${x3},${y3} `} fill="black" />;
};

const useStyles = makeStyles(() => ({
  path: {
    stroke: '#93bfec',
    'stroke-linecap': 'round',
    animation: `$dash 3s ease-in-out infinite`,
  },
}));
