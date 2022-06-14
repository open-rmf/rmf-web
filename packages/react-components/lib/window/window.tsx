import CloseIcon from '@mui/icons-material/Close';
import { Grid, IconButton, Paper, PaperProps, styled } from '@mui/material';
import React from 'react';
import { Layout } from 'react-grid-layout';
import 'react-grid-layout/css/styles.css';
import { WindowManagerStateContext } from './context';
import { WindowToolbar } from './window-toolbar';

export interface WindowProps extends PaperProps {
  key: string;
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
      const windowManagerState = React.useContext(WindowManagerStateContext);
      return (
        <Paper
          ref={ref}
          variant="outlined"
          sx={{ cursor: windowManagerState.designMode ? 'move' : undefined, ...sx }}
          {...otherProps}
        >
          <Grid item>
            <WindowToolbar title={title}>
              {toolbar}
              {windowManagerState.designMode && (
                <IconButton color="inherit" onClick={() => onClose && onClose()}>
                  <CloseIcon />
                </IconButton>
              )}
            </WindowToolbar>
          </Grid>
          {/* NOTE: The resize marker injected by `react-grid-layout` must be a direct children */}
          {children}
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
