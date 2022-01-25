import { AppBar, AppBarProps, styled } from '@mui/material';
import clsx from 'clsx';
import React from 'react';

export type HeaderBarProps = React.PropsWithChildren<AppBarProps>;

const classes = {
  root: 'header-bar-root',
};
const StyledAppBar = styled((props: HeaderBarProps) => <AppBar {...props} />)(() => ({
  [`&.${classes.root}`]: {
    display: 'flex',
    flexDirection: 'row',
    width: '100%',
  },
}));

export const HeaderBar = ({
  id = 'appbar',
  position = 'static',
  className,
  children,
}: HeaderBarProps): React.ReactElement => {
  return (
    <StyledAppBar id={id} position={position} className={clsx(classes.root, className)}>
      {children}
    </StyledAppBar>
  );
};
