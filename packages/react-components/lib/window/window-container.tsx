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

type ResponsiveLayout = { [k in Breakpoint]?: Layout[] };

const Responsive = WidthProvider(Responsive_);

const useStyles = makeStyles({
  windowContainer: {
    overflow: 'auto',
  },
});

export interface WindowContainerProps
  extends Omit<React.PropsWithChildren<ResponsiveProps>, 'layouts'>,
    WidthProviderProps {
  layouts?: ResponsiveLayout | Layout[];
}

export const WindowContainer = React.forwardRef(
  (
    { layouts, className, children, ...otherProps }: WindowContainerProps,
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
    const convertedLayout = React.useMemo(() => {
      if (Array.isArray(layouts)) {
        return theme.breakpoints.keys.reduce(
          (acc, k) => ((acc[k] = layouts), acc),
          {} as ResponsiveLayout,
        );
      }
      return layouts;
    }, [theme.breakpoints.keys, layouts]);
    const classes = useStyles();
    return (
      <Responsive
        ref={ref}
        className={clsx(classes.windowContainer, className)}
        breakpoints={theme.breakpoints.values}
        margin={[theme.spacing(2), theme.spacing(2)]}
        cols={cols}
        rowHeight={32}
        layouts={convertedLayout}
        {...otherProps}
      >
        {children}
      </Responsive>
    );
  },
);

export default WindowContainer;
