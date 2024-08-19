import React from 'react';
import { WindowContainer, WindowLayout } from 'react-components';

export interface WindowState {
  layout: Omit<WindowLayout, 'i'>;
  component: React.ComponentType;
}

export interface WorkspaceState {
  windows: Record<string, WindowState>;
}

export interface WorkspaceProps {
  state: WorkspaceState;
  onStateChange: (state: WorkspaceState) => void;
}

export const Workspace = ({ state, onStateChange }: WorkspaceProps) => {
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
    >
      {Object.entries(state.windows).map(([k, w]) => (
        <w.component key={k} />
      ))}
    </WindowContainer>
  );
};

export interface StaticWorkspaceProps {
  initialState: WorkspaceState;
}

export const StaticWorkspace = ({ initialState }: StaticWorkspaceProps) => {
  const [wsState, setWsState] = React.useState(initialState);
  return <Workspace state={wsState} onStateChange={setWsState} />;
};
