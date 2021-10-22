import { Breakpoint } from '@material-ui/core/styles/createBreakpoints';
import React from 'react';
import { WindowLayout, WindowManagerProps, WindowProps } from 'react-components';
import { v4 as uuidv4 } from 'uuid';

export type ManagedWindowProps = Omit<WindowProps, 'title' | 'children'>;
export type ManagedWindow = React.ComponentType<ManagedWindowProps>;

declare namespace WindowClass {
  type BaseLayout = Omit<WindowLayout, 'i'>;
  type ResponsiveBaseLayouts = {
    [k in keyof Required<WindowManagerProps>['layouts']]: WindowClass.BaseLayout;
  };
}

export class WindowClass {
  name: string;
  Component: ManagedWindow;
  baseLayout: WindowClass.BaseLayout;
  responsiveBaseLayouts: WindowClass.ResponsiveBaseLayouts;

  constructor(
    name: string,
    Component: ManagedWindow,
    baseLayout: WindowClass.BaseLayout,
    responsiveBaseLayouts: WindowClass.ResponsiveBaseLayouts = {},
  ) {
    this.name = name;
    this.Component = Component;
    this.baseLayout = baseLayout;
    this.responsiveBaseLayouts = responsiveBaseLayouts;
  }

  createLayout(breakpoint: Breakpoint, layout?: Partial<WindowLayout>): WindowLayout {
    const base = this.responsiveBaseLayouts[breakpoint] || this.baseLayout;
    return {
      i: uuidv4(),
      ...base,
      ...layout,
    };
  }
}

export const allWindows: Record<string, WindowClass> = {};
