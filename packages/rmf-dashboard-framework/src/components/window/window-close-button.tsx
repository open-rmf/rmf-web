import CloseIcon from '@mui/icons-material/Close';
import { IconButton, IconButtonProps } from '@mui/material';
import React from 'react';

import { WindowManagerStateContext } from './context';

export const WindowCloseButton = ({ className, sx, ...otherProps }: IconButtonProps) => {
  const windowManagerState = React.useContext(WindowManagerStateContext);

  return (
    windowManagerState.designMode && (
      <IconButton
        color="inherit"
        className={`window-toolbar-close ${className}`.trimEnd()}
        sx={{ pointerEvents: 'auto', ...sx }}
        {...otherProps}
      >
        <CloseIcon />
      </IconButton>
    )
  );
};

export default WindowCloseButton;
