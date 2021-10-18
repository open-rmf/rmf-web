import { Grid, IconButton, makeStyles, Paper, PaperProps } from '@material-ui/core';
import CloseIcon from '@material-ui/icons/Close';
import clsx from 'clsx';
import React from 'react';
import { Layout } from 'react-grid-layout';
import 'react-grid-layout/css/styles.css';
import { WindowManagerStateContext } from './context';
import WindowToolbar from './window-toolbar';

const useStyles = makeStyles({
  window: {
    display: 'flex',
    flexDirection: 'column',
    flexWrap: 'nowrap',
  },
});

export interface WindowProps extends PaperProps {
  key: string;
  title: string;
  'data-grid'?: Layout;
  toolbar?: React.ReactNode;
  onClose?: () => void;
}

export const Window: React.FC<WindowProps> = React.forwardRef(
  ({ title, toolbar, onClose, className, children, style, ...otherProps }, ref) => {
    const windowManagerState = React.useContext(WindowManagerStateContext);
    const classes = useStyles();
    return (
      <Paper
        ref={ref}
        variant="outlined"
        className={clsx(classes.window, className)}
        style={{ cursor: windowManagerState.designMode ? 'move' : undefined, ...style }}
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
);

export default Window;
