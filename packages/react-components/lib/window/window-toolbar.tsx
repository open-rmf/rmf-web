import { AppBar, AppBarProps, Box, Grid, Toolbar, Typography } from '@mui/material';
import React from 'react';

export interface WindowToolbarProps extends AppBarProps {
  title: string;
  toolbarItemContainerProps?: React.ComponentProps<typeof Box>;
}

export const WindowToolbar: React.FC<WindowToolbarProps> = ({
  title,
  toolbarItemContainerProps,
  children,
  ...otherProps
}) => {
  return (
    <AppBar position="static" elevation={0} {...otherProps}>
      <Toolbar variant="dense" disableGutters sx={{ paddingLeft: 2 }}>
        <Grid container justifyContent="space-between" alignItems="center">
          <Typography variant="h6">{title}</Typography>
          <Box {...toolbarItemContainerProps}>{children}</Box>
        </Grid>
      </Toolbar>
    </AppBar>
  );
};
