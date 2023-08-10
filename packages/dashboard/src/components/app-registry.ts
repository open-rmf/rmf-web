import { BeaconsApp } from './beacons-app';
import { DoorsApp } from './doors-app';
import { LiftsApp } from './lifts-app';
import { MapApp } from './map-app';
import { RobotInfoApp } from './robots/robot-info-app';
import { RobotsApp } from './robots/robots-app';
import { TaskDetailsApp } from './tasks/task-details-app';
import { TaskLogsApp } from './tasks/task-logs-app';
import { TasksApp } from './tasks/tasks-app';

export const AppRegistry = {
  Beacons: BeaconsApp,
  Doors: DoorsApp,
  Lifts: LiftsApp,
  Map: MapApp,
  Tasks: TasksApp,
  'Task Details': TaskDetailsApp,
  'Task Logs': TaskLogsApp,
  Robots: RobotsApp,
  'Robot Info': RobotInfoApp,
};
