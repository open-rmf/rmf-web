import { WorkspaceState } from '../workspace';

export const robotsWorkspace: WorkspaceState = {
  layout: [
    { i: 'robots', x: 0, y: 0, w: 7, h: 4 },
    { i: 'map', x: 8, y: 0, w: 5, h: 8 },
    { i: 'doors', x: 0, y: 0, w: 7, h: 4 },
    { i: 'lifts', x: 0, y: 0, w: 7, h: 4 },
    { i: 'mutexGroups', x: 8, y: 0, w: 5, h: 4 },
  ],
  windows: [
    { key: 'robots', appName: 'Robots' },
    { key: 'map', appName: 'Map' },
    { key: 'doors', appName: 'Doors' },
    { key: 'lifts', appName: 'Lifts' },
    { key: 'mutexGroups', appName: 'Mutex Groups' },
  ],
};
