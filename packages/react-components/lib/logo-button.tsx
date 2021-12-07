import { ButtonBase, ButtonBaseProps, styled } from '@mui/material';
import clsx from 'clsx';
import React from 'react';

const classes = {
  logoBtn: 'logo-button-root',
  logoImg: 'logo-button-image',
};
const StyledButtonBase = styled((props: ButtonBaseProps) => <ButtonBase {...props} />)(
  ({ theme }) => ({
    [`&.${classes.logoBtn}`]: {
      padding: `${theme.spacing(1)} ${theme.spacing(2)}`,
      boxSizing: 'border-box',
    },
    [`& .${classes.logoImg}`]: {
      width: '100%',
      height: '100%',
    },
  }),
);

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
  return (
    <StyledButtonBase className={clsx(classes.logoBtn, className)} disableRipple {...otherProps}>
      <img src={src} alt={alt} className={classes.logoImg} />
    </StyledButtonBase>
  );
};
