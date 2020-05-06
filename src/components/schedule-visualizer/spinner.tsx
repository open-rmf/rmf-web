import React from 'react';
import { makeStyles } from '@material-ui/core';

export interface SpinnerProps {
  cx: number;
  cy: number;
  r: number;
  strokeWidth?: number;
}

export const Spinner = (props: SpinnerProps) => {
  const { cx, cy, r, strokeWidth } = props;
  const classes = useStyles();
  return (
    <circle
      className={classes.path}
      cx={cx}
      cy={cy}
      r={r}
      fill="none"
      stroke-width={strokeWidth || 0.1}
    ></circle>
  );
};

const useStyles = makeStyles(() => ({
  path: {
    stroke: '#93bfec',
    'stroke-linecap': 'round',
    animation: `$dash 3s ease-in-out infinite`,
  },

  '@keyframes rotate': {
    '0%': {
      transform: 'rotateZ(0deg)',
    },
    '100%': {
      transform: 'rotate(360deg)',
    },
  },

  '@keyframes dash': {
    '0%': {
      strokeDasharray: '1, 150',
      strokeDashoffset: '0',
    },
    '50%': {
      strokeDasharray: '90, 150',
      strokeDashoffset: '-35',
    },
    '100%': {
      strokeDasharray: '90, 180',
      strokeDashoffset: '-124',
    },
  },
}));
