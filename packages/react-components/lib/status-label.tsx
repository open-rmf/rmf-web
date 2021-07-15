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
    width: '5rem',
    textAlign: 'center',
    flexShrink: 0,
  },
  unknown: {
    borderColor: theme.palette.grey[500],
  },
}));

export interface StatusLabelProps {
  /**
   * The text to show on the label. Because the label has a fixed width, the string should not
   * have more than 6-9 characters, depending on the font and character width.
   */
  text?: string;
  className?: string;

  /**
   * Defaults to `normal`.
   *
   * The `unknown` variant should be used when displaying an unknown status. When using the
   * `unknown` variant, the `borderColor` css and `text` props are ignored.
   */
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
