import { AppBar, AppBarProps, Toolbar, Typography } from '@material-ui/core';
import React from 'react';

export interface WindowToolbarProps extends AppBarProps {
  title: string;
  disableMove?: boolean;
}

export const WindowToolbar: React.FC<WindowToolbarProps> = ({
  title,
  disableMove,
  style,
  children,
  ...otherProps
}) => {
  return (
    <AppBar
      position="static"
      elevation={0}
      style={{ cursor: disableMove ? undefined : 'move', ...style }}
      {...otherProps}
    >
      <Toolbar variant="dense" style={{ paddingRight: 0 }}>
        <Typography variant="h6" style={{ flexGrow: 1 }}>
          {title}
        </Typography>
        {children}
      </Toolbar>
    </AppBar>
  );
};

export default WindowToolbar;
