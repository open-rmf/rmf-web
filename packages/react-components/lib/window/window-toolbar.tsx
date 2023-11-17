import { AppBar, AppBarProps, Toolbar, Typography, useMediaQuery } from '@mui/material';
import React from 'react';

export interface WindowToolbarProps extends AppBarProps {
  title: string;
}

export const WindowToolbar: React.FC<WindowToolbarProps> = ({ title, children, ...otherProps }) => {
  const isSmallerThan1000 = useMediaQuery('(max-width:1000px)');
  return (
    <AppBar
      position="static"
      elevation={0}
      style={{ height: isSmallerThan1000 ? 30 : 50 }}
      {...otherProps}
    >
      <Toolbar variant="dense" style={{ paddingRight: 0 }}>
        <Typography
          fontSize={`${isSmallerThan1000 ? '1rem' : '1.7rem'}`}
          style={{ flexGrow: 1 }}
          mb={isSmallerThan1000 ? 2 : 0}
        >
          {title}
        </Typography>
        {children}
      </Toolbar>
    </AppBar>
  );
};
