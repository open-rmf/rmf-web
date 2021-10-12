import { Grid, makeStyles, Paper, PaperProps } from '@material-ui/core';
import clsx from 'clsx';
import React from 'react';
import { Layout } from 'react-grid-layout';
import 'react-grid-layout/css/styles.css';

const useStyles = makeStyles({
  window: {
    display: 'flex',
    flexDirection: 'column',
    flexWrap: 'nowrap',
  },
});

export interface WindowProps extends PaperProps {
  key: string;
  'data-grid'?: Layout;
  header?: React.ReactNode;
}

export const Window: React.FC<WindowProps> = React.forwardRef(
  ({ header, className, children, ...otherProps }, ref) => {
    const classes = useStyles();
    return (
      <Paper
        ref={ref}
        variant="outlined"
        className={clsx(classes.window, className)}
        {...otherProps}
      >
        {header && <Grid item>{header}</Grid>}
        {/* NOTE: The resize marker injected by `react-grid-layout` must be a direct children */}
        {children}
      </Paper>
    );
  },
);

export default Window;
