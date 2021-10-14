import React from 'react';

export interface WindowContainerState {
  designMode: boolean;
}

export const WindowContainerStateContext = React.createContext<WindowContainerState>({
  designMode: false,
});
