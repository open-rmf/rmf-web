import { AppBar, AppBarProps, Box, Grid, Toolbar, Typography } from '@mui/material';
import React from 'react';

export interface WindowToolbarProps extends AppBarProps {
  title: string;
}

export const WindowToolbar: React.FC<WindowToolbarProps> = ({ title, children, ...otherProps }) => {
  return (
    <AppBar position="static" elevation={0} {...otherProps}>
      <Toolbar variant="dense" disableGutters sx={{ paddingLeft: 2 }}>
        <Grid container justifyContent="space-between" alignItems="center">
          <Typography variant="h6">{title}</Typography>
          <Box>{children}</Box>
        </Grid>
      </Toolbar>
    </AppBar>
  );
};
