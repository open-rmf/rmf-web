import React from 'react';
import ProgressBar from 'react-customizable-progressbar';
import { createStyles, makeStyles } from '@material-ui/styles';

export interface CircularProgressBarProps {
  progress: number;
  strokeColor: string;
  children?: React.ReactNode;
}

const useStyles = makeStyles(() =>
  createStyles({
    indicator: {
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
  }),
);

export function CircularProgressBar(props: CircularProgressBarProps): JSX.Element {
  const { progress, strokeColor, children } = props;
  const classes = useStyles();

  return (
    <ProgressBar
      radius={60}
      progress={progress}
      cut={120}
      rotate={-210}
      strokeColor={strokeColor}
      strokeWidth={10}
      trackStrokeWidth={10}
    >
      <div className={classes.indicator}>{children}</div>
    </ProgressBar>
  );
}
