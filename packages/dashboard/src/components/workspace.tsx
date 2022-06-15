import React from 'react';
import { WindowContainer, WindowLayout } from 'react-components';
import { MicroAppProps } from './micro-app';

export interface WorkspaceWindow {
  key: string;
  app: React.ComponentType<MicroAppProps>;
}

export interface WorkspaceState {
  layout: WindowLayout[];
  windows: WorkspaceWindow[];
}

export interface WorkspaceProps {
  state: WorkspaceState;
  onStateChange?: (state: WorkspaceState) => void;
}

export function Workspace({ state, onStateChange }: WorkspaceProps): JSX.Element {
  return (
    <WindowContainer
      layout={state.layout}
      onLayoutChange={(newLayout) =>
        onStateChange && onStateChange({ ...state, layout: newLayout })
      }
      designMode
    >
      {state.windows.map((w) => (
        <w.app
          key={w.key}
          onClose={() => {
            onStateChange &&
              onStateChange({
                layout: state.layout.filter((l) => l.i !== w.key),
                windows: state.windows.filter((w2) => w2.key !== w.key),
              });
          }}
        />
      ))}
    </WindowContainer>
  );
}

export const WorkspaceManager: Record<string, WorkspaceState> = {};
