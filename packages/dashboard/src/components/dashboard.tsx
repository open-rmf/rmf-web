import { DoorsApp } from './doors-app';
import { WorkspaceManager, WorkspaceProps } from './workspace';

export const dashboardWorkspace: WorkspaceProps = {
  layout: [{ i: 'doors', x: 0, y: 0, w: 4, h: 4 }],
  windows: [{ key: 'doors', app: DoorsApp }],
};

WorkspaceManager['dashboard'] = dashboardWorkspace;
