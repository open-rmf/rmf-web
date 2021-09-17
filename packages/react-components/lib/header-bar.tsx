import { AppBar, AppBarProps, styled } from '@material-ui/core';
import clsx from 'clsx';
import React from 'react';

export type HeaderBarProps = React.PropsWithChildren<AppBarProps>;

const classes = {
  root: 'header-bar-root',
};
const HeaderBarRoot = styled((props: HeaderBarProps) => <AppBar {...props} />)(() => ({
  [`&.${classes.root}`]: {
    display: 'flex',
    flexDirection: 'row',
    alignItems: 'center',
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
    <HeaderBarRoot id={id} position={position} className={clsx(classes.root, className)}>
      {children}
    </HeaderBarRoot>
  );
};
