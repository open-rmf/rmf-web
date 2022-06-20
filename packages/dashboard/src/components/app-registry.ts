import { DoorsApp } from './doors-app';
import { LiftsApp } from './lifts-app';
import { MapApp } from './map-app';
import { TaskDetailsApp } from './tasks/task-details-app';
import { TaskLogsApp } from './tasks/task-logs-app';
import { TasksApp } from './tasks/tasks-app';

export const AppRegistry = {
  Doors: DoorsApp,
  Lifts: LiftsApp,
  Map: MapApp,
  Tasks: TasksApp,
  'Task Details': TaskDetailsApp,
  'Task Logs': TaskLogsApp,
};
