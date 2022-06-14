import React from 'react';

/**
 * A window is a react component with no props.
 */
export type Window = React.ComponentType<{}>;

export class WindowManager {
  public windows: Record<string, Window> = {};
}

export const GlobalWindowManager = new WindowManager();
