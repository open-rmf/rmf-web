import React from 'react';

export interface WindowManagerState {
  designMode: boolean;
}

export const WindowManagerStateContext = React.createContext<WindowManagerState>({
  designMode: false,
});
