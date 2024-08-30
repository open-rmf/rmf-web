import type {} from '@emotion/styled';
import CloseIcon from '@mui/icons-material/Close';
import { Box, IconButton, Paper, PaperProps, styled, useTheme } from '@mui/material';
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

      // The resize marker injected by `react-grid-layout` must be a direct children, but we
      // want to wrap the window components in a div so it shows a scrollbar. So we assume that
      // the injected resize marker is always the last component and render it separately.
      const childrenArr = React.Children.toArray(children);
      const childComponents = childrenArr.slice(0, childrenArr.length - 1);
      const resizeComponent = childrenArr[childrenArr.length - 1];

      const windowManagerState = React.useContext(WindowManagerStateContext);
      return (
        <Paper
          ref={ref}
          variant="outlined"
          sx={{
            cursor: windowManagerState.designMode ? 'move' : undefined,
            borderRadius: theme.shape.borderRadius,
            '& > :not(.custom-resize-handle)': {
              pointerEvents: windowManagerState.designMode ? 'none' : undefined,
            },
            '& .window-toolbar-items': {
              pointerEvents: 'auto',
            },
            ...sx,
          }}
          {...otherProps}
        >
          <WindowToolbar
            title={title}
            toolbarItemContainerProps={{ className: 'window-toolbar-items' }}
          >
            {toolbar}
            {windowManagerState.designMode && (
              <IconButton color="inherit" onClick={() => onClose && onClose()}>
                <CloseIcon />
              </IconButton>
            )}
          </WindowToolbar>
          <Box width="100%" height="100%" overflow="auto">
            {childComponents}
          </Box>
          {windowManagerState.designMode && resizeComponent}
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
