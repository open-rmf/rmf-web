import React from 'react';
import { WindowContainer, WindowLayout } from 'react-components';
import { MicroAppProps } from './micro-app';

export interface WorkspaceWindow {
  key: string;
  app: React.ComponentType<MicroAppProps>;
}

export interface WorkspaceProps {
  layout: WindowLayout[];
  windows: WorkspaceWindow[];
}

export function Workspace({ layout, windows }: WorkspaceProps): JSX.Element {
  return (
    <WindowContainer layout={layout} designMode>
      {windows.map((w) => (
        <w.app key={w.key} />
      ))}
    </WindowContainer>
  );
}

export const WorkspaceManager: Record<string, WorkspaceProps> = {};
