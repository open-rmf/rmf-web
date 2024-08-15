import 'react-grid-layout/css/styles.css';
import './no-rgl-animations.css';

import React from 'react';
import { default as GridLayout_, Layout as WindowLayout, WidthProvider } from 'react-grid-layout';

import { WindowManagerStateContext } from './context';

export type { Layout as WindowLayout } from 'react-grid-layout';

const GridLayout = WidthProvider(GridLayout_);
// TODO: mui 5 switched theme spacing to string, instead of converting css units to px, just use
// a fixed margin for now.
const MARGIN = 8;

export type WindowContainerProps = React.PropsWithChildren<{
  /**
   * Layout works by splitting the available space into 12 columns and rows.
   * The values defined in the layouts are in column / row units. The size of each
   * column and row is automatically adjusted based on the size of the container.
   */
  layout: WindowLayout[];

  cols?: number;

  /**
   * Enables dragging and resizing of windows.
   */
  designMode?: boolean;
  onLayoutChange?: (newLayout: WindowLayout[]) => void;
}>;

/**
 * A resizable and draggable grid layout based on react-grid-layout. By default, the layout
 * follows material-ui convention of 12 columns and rows.
 */
export const WindowContainer: React.FC<WindowContainerProps> = ({
  layout,
  cols = 12,
  designMode = false,
  onLayoutChange,
  children,
}: WindowContainerProps) => {
  const windowManagerState = React.useMemo(
    () => ({
      designMode: designMode ?? false,
    }),
    [designMode],
  );

  return (
    <WindowManagerStateContext.Provider value={windowManagerState}>
      <GridLayout
        layout={layout}
        margin={[MARGIN, MARGIN]}
        cols={cols}
        compactType={null}
        preventCollision
        isResizable={designMode}
        isDraggable={designMode}
        draggableHandle=".rgl-draggable"
        onLayoutChange={onLayoutChange}
      >
        {children}
      </GridLayout>
    </WindowManagerStateContext.Provider>
  );
};
