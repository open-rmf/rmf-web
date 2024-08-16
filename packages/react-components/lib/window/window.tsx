import type {} from '@emotion/styled';
import CloseIcon from '@mui/icons-material/Close';
import { Box, Grid, IconButton, Paper, PaperProps, styled, useTheme } from '@mui/material';
import React from 'react';
import { Layout } from 'react-grid-layout';

import { WindowManagerStateContext } from './context';
import { WindowToolbar } from './window-toolbar';

export interface WindowProps extends PaperProps {
  title: string;
  'data-grid'?: Layout;
  toolbar?: React.ReactNode;
  onClose?: () => void;
}

export const Window = styled(
  React.forwardRef(
    (
      { title, toolbar, onClose, sx, children, ...otherProps }: WindowProps,
      ref: React.Ref<HTMLDivElement>,
    ) => {
      const theme = useTheme();

      const windowManagerState = React.useContext(WindowManagerStateContext);
      return (
        <Paper
          ref={ref}
          variant="outlined"
          sx={{
            cursor: windowManagerState.designMode ? 'move' : undefined,
            borderRadius: theme.shape.borderRadius,
            ...sx,
          }}
          {...otherProps}
        >
          <Grid item className="rgl-draggable">
            <WindowToolbar title={title}>
              {toolbar}
              {windowManagerState.designMode && (
                <IconButton color="inherit" onClick={() => onClose && onClose()}>
                  <CloseIcon />
                </IconButton>
              )}
            </WindowToolbar>
          </Grid>
          <Box sx={{ overflow: 'auto', width: '100%', height: '100%', cursor: 'auto' }}>
            {children}
          </Box>
        </Paper>
      );
    },
  ),
)({
  display: 'flex',
  flexDirection: 'column',
  flexWrap: 'nowrap',
  overflow: 'hidden',
});
