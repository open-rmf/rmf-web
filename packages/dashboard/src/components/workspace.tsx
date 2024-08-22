import React from 'react';
import { WindowContainer, WindowLayout } from 'react-components';

import { MicroAppProps } from './micro-app';

export interface WindowState {
  layout: Omit<WindowLayout, 'i'>;
  Component: React.ComponentType<MicroAppProps>;
}

export interface WorkspaceState {
  windows: Record<string, WindowState>;
}

export interface WorkspaceProps {
  state: WorkspaceState;
  onStateChange: (state: WorkspaceState) => void;
  designMode?: boolean;
}

export const Workspace = ({ state, onStateChange, designMode = false }: WorkspaceProps) => {
  const layout: WindowLayout[] = Object.entries(state.windows).map(([k, w]) => ({
    i: k,
    ...w.layout,
  }));

  return (
    <WindowContainer
      layout={layout}
      onLayoutChange={(newLayout) => {
        const newState = { ...state };
        newLayout.forEach((l) => (newState.windows[l.i].layout = l));
        onStateChange(newState);
      }}
      designMode={designMode}
    >
      {Object.entries(state.windows).map(([k, w]) => (
        <w.Component
          key={k}
          onClose={() => {
            const newState = { ...state };
            delete newState.windows[k];
            onStateChange(newState);
          }}
        />
      ))}
    </WindowContainer>
  );
};

export interface StaticWorkspaceProps {
  /**
   * Initial state of the workspace. This is only used on the first render, changes on this
   * after the initial render will not do anything.
   */
  initialState: WorkspaceState;
}

/**
 * A simple predefined workspace where the layout is fixed.
 */
export const StaticWorkspace = ({ initialState }: StaticWorkspaceProps) => {
  const [wsState, setWsState] = React.useState(initialState);
  return <Workspace state={wsState} onStateChange={setWsState} />;
};
