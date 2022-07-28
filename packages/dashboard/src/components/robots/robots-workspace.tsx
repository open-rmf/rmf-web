import { WorkspaceState } from '../workspace';

export const robotsWorkspace: WorkspaceState = {
  layout: [
    { i: 'robots', x: 2, y: 0, w: 6, h: 12 },
    { i: 'robots-info', x: 8, y: 0, w: 2, h: 6 },
    { i: 'map', x: 8, y: 6, w: 2, h: 6 },
  ],
  windows: [
    { key: 'robots', appName: 'Robots' },
    { key: 'robots-info', appName: 'Robot Info' },
    { key: 'map', appName: 'Map' },
  ],
};
