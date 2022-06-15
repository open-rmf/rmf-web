import { DoorsApp } from './doors-app';
import { LiftsApp } from './lifts-app';
import { WorkspaceManager, WorkspaceState } from './workspace';

export const dashboardWorkspace: WorkspaceState = {
  layout: [
    { i: 'doors', x: 0, y: 0, w: 4, h: 4 },
    { i: 'lifts', x: 4, y: 0, w: 4, h: 4 },
  ],
  windows: [
    { key: 'doors', app: DoorsApp },
    { key: 'lifts', app: LiftsApp },
  ],
};

WorkspaceManager['dashboard'] = dashboardWorkspace;
