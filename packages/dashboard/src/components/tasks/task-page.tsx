/* istanbul ignore file */

import { makeStyles } from '@material-ui/core';
import type { TaskProgress } from 'api-client';
import React from 'react';
import { TaskPanel, TaskPanelProps } from 'react-components';
import { PlacesContext, RmfIngressContext } from '../rmf-app';

const useStyles = makeStyles((theme) => ({
  taskPanel: {
    margin: `${theme.spacing(4)}px auto`,
    width: '100%',
    height: '100%',
    maxWidth: 1600,
  },
}));

export function TaskPage() {
  const classes = useStyles();
  const { tasksApi = null } = React.useContext(RmfIngressContext) || {};
  const places = React.useContext(PlacesContext);

  const fetchTasks = React.useCallback<TaskPanelProps['fetchTasks']>(
    async (limit: number, offset: number) => {
      if (!tasksApi) {
        return {
          tasks: [],
          totalCount: 0,
        };
      }
      const resp = await tasksApi.getTasksTasksGet(
        undefined,
        undefined,
        undefined,
        undefined,
        undefined,
        undefined,
        undefined,
        undefined,
        undefined,
        limit,
        offset,
        '-priority,-start_time',
      );
      const taskProgresses: TaskProgress[] = resp.data.items;
      const task_summaries = taskProgresses.map((t) => t.task_summary);
      return {
        tasks: task_summaries,
        totalCount: resp.data.total_count,
      };
    },
    [tasksApi],
  );

  const submitTasks = React.useCallback<Required<TaskPanelProps>['submitTasks']>(
    async (tasks) => {
      if (!tasksApi) {
        throw new Error('tasks api not available');
      }
      for (const t of tasks) {
        await tasksApi.submitTaskTasksSubmitTaskPost(t);
      }
    },
    [tasksApi],
  );

  const cancelTask = React.useCallback<Required<TaskPanelProps>['cancelTask']>(
    async (task) => {
      await tasksApi?.cancelTaskTasksCancelTaskPost({ task_id: task.task_id });
    },
    [tasksApi],
  );

  return (
    <TaskPanel
      className={classes.taskPanel}
      fetchTasks={fetchTasks}
      cleaningZones={Object.keys(places)}
      loopWaypoints={Object.keys(places)}
      deliveryWaypoints={Object.keys(places)}
      submitTasks={submitTasks}
      cancelTask={cancelTask}
    />
  );
}
