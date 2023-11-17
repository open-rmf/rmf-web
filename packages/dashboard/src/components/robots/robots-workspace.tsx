import { WorkspaceState } from '../workspace';

export const robotsWorkspace: WorkspaceState = {
  layout: [
    { i: 'robots', x: 0, y: 0, w: 7, h: 4 },
    { i: 'map', x: 8, y: 0, w: 5, h: 12 },
    { i: 'doors', x: 0, y: 0, w: 7, h: 3 },
    { i: 'lifts', x: 0, y: 0, w: 7, h: 3 },
    { i: 'beacons', x: 0, y: 0, w: 7, h: 2 },
  ],
  windows: [
    { key: 'robots', appName: 'Robots' },
    { key: 'map', appName: 'Map' },
    { key: 'doors', appName: 'Doors' },
    { key: 'lifts', appName: 'Lifts' },
    { key: 'beacons', appName: 'Beacons' },
  ],
};
