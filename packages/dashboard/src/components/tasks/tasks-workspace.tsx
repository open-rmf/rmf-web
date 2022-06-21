import { WorkspaceState } from '../workspace';

export const tasksWorkspace: WorkspaceState = {
  layout: [
    { i: 'tasks', x: 2, y: 0, w: 6, h: 12 },
    { i: 'task-details', x: 8, y: 0, w: 2, h: 6 },
    { i: 'task-logs', x: 8, y: 6, w: 2, h: 6 },
  ],
  windows: [
    { key: 'tasks', appName: 'Tasks' },
    { key: 'task-details', appName: 'Task Details' },
    { key: 'task-logs', appName: 'Task Logs' },
  ],
};
