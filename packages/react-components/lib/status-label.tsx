import { Typography, styled } from '@mui/material';
import clsx from 'clsx';
import React from 'react';

const classes = {
  status: 'status-label-root',
  unknown: 'status-label-unknown',
};
const StyledDiv = styled('div')(({ theme }) => ({
  [`&.${classes.status}`]: {
    borderColor: theme.palette.primary.main,
    borderRadius: theme.shape.borderRadius,
    borderStyle: 'solid',
    borderWidth: '2px',
    padding: '5px',
    width: '4rem',
    textAlign: 'center',
    flexShrink: 0,
  },
  [`&.${classes.unknown}`]: {
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
  return (
    <StyledDiv
      className={clsx(
        classes.status,
        className,
        variant === 'unknown' ? classes.unknown : undefined,
      )}
      {...otherProps}
    >
      <Typography variant="button" role="status">
        {variant === 'unknown' ? 'N/A' : text}
      </Typography>
    </StyledDiv>
  );
};
