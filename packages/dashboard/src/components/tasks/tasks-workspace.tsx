import { WorkspaceState } from '../workspace';

export const tasksWorkspace: WorkspaceState = {
  layout: [
    { i: 'tasks', x: 1, y: 0, w: 6, h: 12 },
    { i: 'task-details', x: 7, y: 0, w: 2, h: 12 },
    { i: 'task-logs', x: 9, y: 0, w: 2, h: 12 },
  ],
  windows: [
    { key: 'tasks', appName: 'Tasks' },
    { key: 'task-details', appName: 'Task Details' },
    { key: 'task-logs', appName: 'Task Logs' },
  ],
};
