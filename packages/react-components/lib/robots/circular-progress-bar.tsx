import React from 'react';
import ProgressBar from 'react-customizable-progressbar';
import { ReactCustomizableProgressbarProps } from 'react-customizable-progressbar';
import { styled } from '@mui/material';

export interface CircularProgressBarProps {
  progress: number;
  strokeColor: string;
  children?: React.ReactNode;
}

const classes = {
  indicator: 'circular-progressbar-indicator',
};
const StyledCircularProgressBar = styled((props: ReactCustomizableProgressbarProps) => (
  <ProgressBar {...props} />
))(() => ({
  [`& .${classes.indicator}`]: {
    display: 'flex',
    flexDirection: 'column',
    justifyContent: 'center',
    textAlign: 'center',
    position: 'absolute',
    top: 0,
    width: '100%',
    height: '100%',
    margin: '0 auto',
  },
}));

export function CircularProgressBar(props: CircularProgressBarProps): JSX.Element {
  const { progress, strokeColor, children } = props;

  return (
    <StyledCircularProgressBar
      radius={60}
      progress={progress}
      cut={120}
      rotate={-210}
      strokeColor={strokeColor}
      strokeWidth={10}
      trackStrokeWidth={10}
    >
      <div className={classes.indicator}>{children}</div>
    </StyledCircularProgressBar>
  );
}
