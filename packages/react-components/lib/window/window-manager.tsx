import { makeStyles, useTheme } from '@material-ui/core';
import { Breakpoint } from '@material-ui/core/styles/createBreakpoints';
import React from 'react';
import { Layout, Responsive as Responsive_, WidthProvider } from 'react-grid-layout';
import 'react-grid-layout/css/styles.css';
import { WindowManagerStateContext } from './context';
import { WindowProps } from './window';

export type WindowLayout = Layout;
type ResponsiveLayouts = { [k in Breakpoint]?: WindowLayout[] };

const Responsive = WidthProvider(Responsive_);

const useStyles = makeStyles({
  windowContainer: {
    overflow: 'auto',
  },
});

export interface WindowManagerProps extends React.HTMLProps<HTMLDivElement> {
  /**
   * Layout is works by splitting the available space into 12 columns and rows.
   * The values defined in the layouts are in column / row units. The size of each
   * column and row is automatically adjusted based on the size of the container.
   */
  layouts?: ResponsiveLayouts;
  /**
   * Enables dragging and resizing of windows.
   */
  designMode?: boolean;
  onLayoutChange?: (newLayouts: ResponsiveLayouts) => void;
  children?: React.ReactElement<WindowProps> | React.ReactElement<WindowProps>[];
}

export const WindowManager: React.FC<WindowManagerProps> = ({
  layouts,
  designMode,
  onLayoutChange,
  children,
  ...otherProps
}: WindowManagerProps) => {
  const theme = useTheme();

  const [mounted, setMounted] = React.useState(false);
  React.useEffect(() => {
    setMounted(true);
  }, []);

  const cols = React.useMemo(
    () =>
      theme.breakpoints.keys.reduce(
        (acc, k) => ((acc[k] = 12), acc), // fixed 12 cols, same as material grid
        {} as Record<Breakpoint, number>,
      ),
    [theme.breakpoints.keys],
  );

  const windowManagerState = React.useMemo(
    () => ({
      designMode: designMode ?? false,
    }),
    [designMode],
  );

  const containerRef = React.useRef<HTMLDivElement>(null);
  const [rowHeight, setRowHeight] = React.useState(32);
  React.useEffect(() => {
    if (!containerRef.current) return;
    const resizeObserver = new ResizeObserver((entries) => {
      const contentSize = entries[0].contentBoxSize[0];
      setRowHeight((contentSize.blockSize - theme.spacing(26)) / 12);
    });
    resizeObserver.observe(containerRef.current);
    return () => {
      resizeObserver.disconnect();
    };
  }, [theme]);

  const handleLayoutChange = React.useCallback(
    (_, newLayouts) => onLayoutChange && onLayoutChange(newLayouts),
    [onLayoutChange],
  );

  const classes = useStyles();

  return (
    <div ref={containerRef} {...otherProps}>
      <WindowManagerStateContext.Provider value={windowManagerState}>
        <Responsive
          className={classes.windowContainer}
          breakpoints={theme.breakpoints.values}
          margin={[theme.spacing(2), theme.spacing(2)]}
          containerPadding={[theme.spacing(2), theme.spacing(2)]}
          cols={cols}
          rowHeight={rowHeight}
          layouts={layouts}
          compactType={null}
          preventCollision
          isResizable={designMode}
          isDraggable={designMode}
          onLayoutChange={handleLayoutChange}
          measureBeforeMount
          useCSSTransforms={!mounted}
        >
          {children}
        </Responsive>
      </WindowManagerStateContext.Provider>
    </div>
  );
};

export default WindowManager;
