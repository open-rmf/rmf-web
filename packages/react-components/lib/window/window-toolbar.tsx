import { AppBar, AppBarProps, Toolbar, Typography, useMediaQuery } from '@mui/material';
import React from 'react';

export interface WindowToolbarProps extends AppBarProps {
  title: string;
}

export const WindowToolbar: React.FC<WindowToolbarProps> = ({ title, children, ...otherProps }) => {
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');
  return (
    <AppBar
      position="static"
      elevation={0}
      style={{ height: isScreenHeightLessThan800 ? 30 : 50 }}
      {...otherProps}
    >
      <Toolbar variant="dense" style={{ paddingRight: 0 }}>
        <Typography style={{ flexGrow: 1 }} mb={isScreenHeightLessThan800 ? 2 : 0} variant="h6">
          {title}
        </Typography>
        {children}
      </Toolbar>
    </AppBar>
  );
};
