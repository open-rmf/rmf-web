import { makeStyles, Typography, TypographyProps } from '@material-ui/core';
import clsx from 'clsx';
import React from 'react';

const useStyles = makeStyles(theme => ({
  disabled: {
    color: theme.palette.action.disabled,
  },
}));

export interface DisableableTypography extends TypographyProps {
  disabled?: boolean;
}

export default function DisableableTypography(props: DisableableTypography): React.ReactElement {
  const { className, disabled, ...otherProps } = props;
  const classes = useStyles();
  return (
    <Typography className={clsx({ [classes.disabled]: disabled, className })} {...otherProps}>
      {props.children}
    </Typography>
  );
}
