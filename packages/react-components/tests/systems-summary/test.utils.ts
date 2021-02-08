import { ItemSummary, Notification } from '../../lib';
import * as RomiCore from '@osrf/romi-js-core-interfaces';

export const itemSummary: ItemSummary = {
  item: 'Door',
  summary: { operational: 0, outOfOrder: 0, idle: 0, charging: 0 },
  spoiltItemList: [],
};

export const notifications: Notification[] = [
  { id: 1, time: 'time', error: 'message', severity: 'High' },
];

export const tasks: RomiCore.TaskSummary[] = [
  {
    task_id: '1',
    state: 0,
    start_time: { sec: 0, nanosec: 0 },
    status: 'state',
    submission_time: { sec: 0, nanosec: 0 },
    end_time: { sec: 0, nanosec: 0 },
  },
  {
    task_id: '2',
    state: 1,
    start_time: { sec: 0, nanosec: 0 },
    status: 'state',
    submission_time: { sec: 0, nanosec: 0 },
    end_time: { sec: 0, nanosec: 0 },
  },
  {
    task_id: '3',
    state: 2,
    start_time: { sec: 0, nanosec: 0 },
    status: 'state',
    submission_time: { sec: 0, nanosec: 0 },
    end_time: { sec: 0, nanosec: 0 },
  },
  {
    task_id: '4',
    state: 3,
    start_time: { sec: 0, nanosec: 0 },
    status: 'state',
    submission_time: { sec: 0, nanosec: 0 },
    end_time: { sec: 0, nanosec: 0 },
  },
];
