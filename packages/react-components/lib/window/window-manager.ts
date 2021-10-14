import { useTheme } from '@material-ui/core';
import React from 'react';
import { Layout } from 'react-grid-layout';
import { v4 as uuidv4 } from 'uuid';
import { WindowProps } from './window';
import { ResponsiveLayout, WindowContainerProps } from './window-container';

export declare namespace WindowPreference {
  type BaseLayout = Pick<Layout, 'w' | 'h' | 'minW' | 'minH' | 'maxW' | 'maxH'>;

  type BaseResponsiveLayout = {
    [k in keyof ResponsiveLayout]?: BaseLayout;
  };
}

export type WindowLayout = ResponsiveLayout extends { [key in keyof ResponsiveLayout]: (infer T)[] }
  ? { [key in keyof ResponsiveLayout]: T }
  : never;

export interface WindowState {
  key: string;
  windowClass: string;
  windowProps: Omit<WindowProps, 'key'>;
  layout: WindowLayout;
}

export class WindowPreference {
  windowClass: string;
  title: string;
  baseLayout: WindowPreference.BaseResponsiveLayout;

  /**
   * @param baseLayout Must at least give layout for the largest breakpoint.
   */
  constructor(
    windowClass: string,
    title: string,
    baseLayout: WindowPreference.BaseResponsiveLayout,
  ) {
    this.windowClass = windowClass;
    this.title = title;
    this.baseLayout = baseLayout;
  }

  createWindow(): WindowState {
    const key = uuidv4();
    return {
      key,
      windowClass: this.windowClass,
      windowProps: { title: this.title },
      layout: Object.entries(this.baseLayout).reduce<WindowLayout>(
        (acc, [bp, layout]) => (
          (acc[bp as keyof WindowLayout] = { i: key, x: 0, y: 0, ...layout } as Layout), acc
        ),
        {},
      ),
    };
  }
}

export interface WindowPreference {
  windowClass: string;
  title: string;
  baseLayout: WindowPreference.BaseResponsiveLayout;
}

export interface WindowManager {
  windows: Record<string, WindowState>;
  layouts: WindowContainerProps['layouts'];
  addWindow(windowState: WindowState): WindowState;
  removeWindow(key: string): void;
  onLayoutChange: WindowContainerProps['onLayoutChange'];
}

export function useWindowManager(
  initialWindows: WindowState[] | (() => WindowState[]) = [],
): WindowManager {
  const windows = React.useMemo(() => {
    const arr = typeof initialWindows === 'function' ? initialWindows() : initialWindows;
    return arr.reduce<Record<string, WindowState>>(
      (acc, windowState) => ((acc[windowState.key] = windowState), acc),
      {},
    );
  }, [initialWindows]);

  const theme = useTheme();
  const [layouts, setLayouts] = React.useState(() => {
    const windowsValues = Object.values(windows);
    return theme.breakpoints.keys.reduce<ResponsiveLayout>((acc, k) => {
      const breakpointLayouts: Layout[] = [];
      windowsValues.forEach((windowState) => {
        const l = windowState.layout[k];
        if (l) {
          breakpointLayouts.push(l);
        }
      });
      acc[k] = breakpointLayouts;
      return acc;
    }, {});
  });

  const addWindow = React.useCallback(
    (windowState) => {
      windows[windowState.key] = windowState;
      setLayouts((prev) => {
        Object.keys(windowState.layout).forEach((k) => {
          const bp = k as keyof ResponsiveLayout;
          const winLayout = windowState.layout[bp];
          if (winLayout) {
            prev[bp] = [...(prev[bp] || []), winLayout];
          }
        });
        return { ...prev };
      });
      return windows[windowState.key];
    },
    [windows],
  );

  const removeWindow = React.useCallback(
    (key) => {
      delete windows[key];
      setLayouts((prev) =>
        Object.entries(prev).reduce<ResponsiveLayout>(
          (acc, [bp, ls]) => (
            (acc[bp as keyof ResponsiveLayout] = (ls as Layout[]).filter((l) => l.i !== key)), acc
          ),
          {},
        ),
      );
    },
    [windows],
  );

  const onLayoutChange = React.useCallback((_, newLayouts) => setLayouts(newLayouts), []);

  const windowManager = React.useMemo<WindowManager>(
    () => ({
      windows,
      layouts,
      addWindow,
      removeWindow,
      onLayoutChange,
    }),
    [layouts, windows, addWindow, removeWindow, onLayoutChange],
  );
  return windowManager;
}
