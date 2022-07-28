import { AppBar, AppBarProps, Toolbar, Typography } from '@mui/material';
import React from 'react';

export interface WindowToolbarProps extends AppBarProps {
  title: string;
}

export const WindowToolbar: React.FC<WindowToolbarProps> = ({ title, children, ...otherProps }) => {
  return (
    <AppBar position="static" elevation={0} {...otherProps}>
      <Toolbar variant="dense" style={{ paddingRight: 0 }}>
        <Typography variant="h6" style={{ flexGrow: 1 }}>
          {title}
        </Typography>
        {children}
      </Toolbar>
    </AppBar>
  );
};
