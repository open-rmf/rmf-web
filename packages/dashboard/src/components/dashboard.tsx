import { DoorsApp } from './doors-app';
import { LiftsApp } from './lifts-app';
import { MapApp } from './map-app';
import { WorkspaceManager, WorkspaceState } from './workspace';

export const dashboardWorkspace: WorkspaceState = {
  layout: [
    { i: 'map', x: 0, y: 0, w: 8, h: 12 },
    { i: 'doors', x: 8, y: 0, w: 4, h: 6 },
    { i: 'lifts', x: 8, y: 6, w: 4, h: 6 },
  ],
  windows: [
    { key: 'map', app: MapApp },
    { key: 'doors', app: DoorsApp },
    { key: 'lifts', app: LiftsApp },
  ],
};

WorkspaceManager['dashboard'] = dashboardWorkspace;
