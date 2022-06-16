import React from 'react';
import { default as GridLayout_, Layout as WindowLayout, WidthProvider } from 'react-grid-layout';
import 'react-grid-layout/css/styles.css';
import { WindowManagerStateContext } from './context';
import { WindowProps } from './window';

export type { Layout as WindowLayout } from 'react-grid-layout';

const GridLayout = WidthProvider(GridLayout_);
// TODO: mui 5 switched theme spacing to string, instead of converting css units to px, just use
// a fixed margin for now.
const MARGIN = 10;

export interface WindowContainerProps extends React.HTMLProps<HTMLDivElement> {
  /**
   * Layout works by splitting the available space into 12 columns and rows.
   * The values defined in the layouts are in column / row units. The size of each
   * column and row is automatically adjusted based on the size of the container.
   */
  layout: WindowLayout[];

  cols?: number;
  rows?: number;

  /**
   * Enables dragging and resizing of windows.
   */
  designMode?: boolean;
  onLayoutChange?: (newLayout: WindowLayout[]) => void;
  children?: React.ReactElement<WindowProps> | React.ReactElement<WindowProps>[];
}

/**
 * A resizable and draggable grid layout based on react-grid-layout. By default, the layout
 * follows material-ui convention of 12 columns and rows.
 */
export const WindowContainer: React.FC<WindowContainerProps> = ({
  layout,
  cols = 12,
  rows = 12,
  designMode = false,
  onLayoutChange,
  children,
  style,
  ...otherProps
}: WindowContainerProps) => {
  const windowManagerState = React.useMemo(
    () => ({
      designMode: designMode ?? false,
    }),
    [designMode],
  );

  const containerRef = React.useRef<HTMLDivElement>(null);
  const [rowHeight, setRowHeight] = React.useState<number | null>(null);
  React.useEffect(() => {
    if (!containerRef.current) return;
    const resizeObserver = new ResizeObserver((entries) => {
      const contentRect = entries[0].contentRect;
      setRowHeight((contentRect.height - MARGIN * 13) / rows);
    });
    resizeObserver.observe(containerRef.current);
    return () => {
      resizeObserver.disconnect();
    };
  }, [rows]);

  return (
    <WindowManagerStateContext.Provider value={windowManagerState}>
      <div
        ref={containerRef}
        style={{ overflow: 'auto', height: '100%', maxHeight: '100%', ...style }}
        {...otherProps}
      >
        {rowHeight !== null && (
          <GridLayout
            layout={layout}
            margin={[MARGIN, MARGIN]}
            cols={cols}
            compactType={null}
            preventCollision
            autoSize={false}
            maxRows={rows}
            rowHeight={rowHeight}
            isResizable={designMode}
            isDraggable={designMode}
            useCSSTransforms={false}
            draggableHandle=".rgl-draggable"
            onLayoutChange={onLayoutChange}
          >
            {children}
          </GridLayout>
        )}
      </div>
    </WindowManagerStateContext.Provider>
  );
};
