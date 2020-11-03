import { makeStyles, Typography } from '@material-ui/core';
import React from 'react';
import { joinClasses } from './css-utils';

const useStyles = makeStyles((theme) => ({
  status: {
    borderColor: theme.palette.primary.main,
    borderRadius: theme.shape.borderRadius,
    borderStyle: 'solid',
    border: 2,
    padding: 5,
    width: '4rem',
    textAlign: 'center',
  },
  unknown: {
    borderColor: theme.palette.grey[500],
  },
}));

export interface StatusLabelProps {
  text?: string;
  className?: string;
  variant?: 'normal' | 'unknown';
}

export const StatusLabel = (props: StatusLabelProps): JSX.Element => {
  const { text = '', className, variant = 'normal', ...otherProps } = props;
  const classes = useStyles();
  return (
    <div
      className={joinClasses(
        classes.status,
        className,
        variant === 'unknown' ? classes.unknown : undefined,
      )}
      {...otherProps}
    >
      <Typography variant="button" role="status">
        {variant === 'unknown' ? 'N/A' : text}
      </Typography>
    </div>
  );
};
