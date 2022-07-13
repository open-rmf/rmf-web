import { WorkspaceState } from './workspace';

export const dashboardWorkspace: WorkspaceState = {
  layout: [
    { i: 'map', x: 0, y: 0, w: 9, h: 12 },
    { i: 'doors', x: 9, y: 0, w: 3, h: 6 },
    { i: 'lifts', x: 9, y: 6, w: 3, h: 6 },
  ],
  windows: [
    { key: 'map', appName: 'Map' },
    { key: 'doors', appName: 'Doors' },
    { key: 'lifts', appName: 'Lifts' },
  ],
};
