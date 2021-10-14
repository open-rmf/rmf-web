import { makeStyles, useTheme } from '@material-ui/core';
import { Breakpoint } from '@material-ui/core/styles/createBreakpoints';
import clsx from 'clsx';
import React from 'react';
import {
  Layout,
  Responsive as Responsive_,
  ResponsiveProps,
  WidthProvider,
  WidthProviderProps,
} from 'react-grid-layout';
import 'react-grid-layout/css/styles.css';
import { WindowContainerStateContext } from './context';
import { WindowProps } from './window';

export type ResponsiveLayout = { [k in Breakpoint]?: Layout[] };

const Responsive = WidthProvider(Responsive_);

const useStyles = makeStyles({
  windowContainer: {
    overflow: 'auto',
  },
});

export interface WindowContainerProps extends Omit<ResponsiveProps, 'layouts'>, WidthProviderProps {
  layouts?: ResponsiveLayout | Layout[];
  /**
   * Enables dragging and resizing of windows.
   */
  designMode?: boolean;
  children?: React.ReactElement<WindowProps> | React.ReactElement<WindowProps>[];
}

export const WindowContainer = React.forwardRef(
  (
    { layouts, designMode, className, children, ...otherProps }: WindowContainerProps,
    ref: React.Ref<Responsive_>,
  ) => {
    const theme = useTheme();

    const cols = React.useMemo(
      () =>
        theme.breakpoints.keys.reduce(
          (acc, k) => ((acc[k] = 12), acc), // fixed 12 cols, same as material grid
          {} as Record<Breakpoint, number>,
        ),
      [theme.breakpoints.keys],
    );

    const largestBreakpoint = React.useMemo(() => {
      const breakpointValues = Object.entries(theme.breakpoints.values);
      return Object.entries(theme.breakpoints.values).reduce<[string, number]>(
        (acc, entry) => (entry[1] > acc[1] ? entry : acc),
        breakpointValues[0],
      )[0];
    }, [theme.breakpoints.values]);

    const convertedLayouts = React.useMemo(() => {
      if (Array.isArray(layouts)) {
        return { [largestBreakpoint]: layouts };
      }
      return layouts;
    }, [largestBreakpoint, layouts]);

    const windowContainerState = React.useMemo(
      () => ({
        designMode: designMode ?? false,
      }),
      [designMode],
    );

    const classes = useStyles();

    return (
      <WindowContainerStateContext.Provider value={windowContainerState}>
        <Responsive
          ref={ref}
          className={clsx(classes.windowContainer, className)}
          breakpoints={theme.breakpoints.values}
          margin={[theme.spacing(2), theme.spacing(2)]}
          cols={cols}
          rowHeight={32}
          layouts={convertedLayouts}
          compactType={null}
          preventCollision
          isResizable={designMode}
          isDraggable={designMode}
          {...otherProps}
        >
          {children}
        </Responsive>
      </WindowContainerStateContext.Provider>
    );
  },
);

export default WindowContainer;
