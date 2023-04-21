import { WorkspaceState } from '../workspace';

export const tasksWorkspace: WorkspaceState = {
  layout: [
    { i: 'tasks', x: 0, y: 0, w: 7, h: 12 },
    { i: 'map', x: 8, y: 0, w: 5, h: 12 },
  ],
  windows: [
    { key: 'tasks', appName: 'Tasks' },
    { key: 'map', appName: 'Map' },
  ],
};
