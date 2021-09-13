import { ButtonBase, ButtonBaseProps } from '@material-ui/core';
import { makeStyles } from '@material-ui/styles';
import clsx from 'clsx';
import React from 'react';

const useStyles = makeStyles((theme) => ({
  logoBtn: {
    padding: `${theme.spacing(1)} ${theme.spacing(2)}`,
    boxSizing: 'border-box',
  },
  logoImg: {
    width: '100%',
    height: '100%',
  },
}));

export interface LogoButtonProps extends ButtonBaseProps {
  src: string;
  alt?: string;
}

export const LogoButton = ({
  src,
  alt,
  className,
  ...otherProps
}: LogoButtonProps): JSX.Element => {
  const classes = useStyles();
  return (
    <ButtonBase className={clsx(classes.logoBtn, className)} disableRipple {...otherProps}>
      <img src={src} alt={alt} className={classes.logoImg} />
    </ButtonBase>
  );
};
