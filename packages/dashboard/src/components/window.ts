import { Breakpoint } from '@material-ui/core/styles/createBreakpoints';
import React from 'react';
import { WindowLayout, WindowProps } from 'react-components';

export type ManagedWindowProps = Omit<WindowProps, 'title' | 'children'>;
export type ManagedWindow = React.ComponentType<ManagedWindowProps>;

export interface WindowClass {
  Component: ManagedWindow;
  createLayout: (breakpoint: Breakpoint, layout?: Partial<WindowLayout>) => WindowLayout;
}

export const allWindows: Record<string, WindowClass> = {};
