/* istanbul ignore file */

import { makeStyles } from '@material-ui/core';
import type { TaskProgress } from 'api-client';
import type { AxiosError } from 'axios';
import React from 'react';
import { TaskPanel, TaskPanelProps } from 'react-components';
import { PlacesContext, RmfIngressContext } from '../rmf-app';

const useStyles = makeStyles((theme) => ({
  taskPanel: {
    padding: `${theme.spacing(4)}px`,
    height: '100%',
    backgroundColor: theme.secondaryBackground,
  },
}));

export function TaskPage() {
  const classes = useStyles();
  const { tasksApi = null } = React.useContext(RmfIngressContext) || {};
  const places = React.useContext(PlacesContext);
  const placeNames = places.map((p) => p.vertex.name);

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
      try {
        await tasksApi?.cancelTaskTasksCancelTaskPost({ task_id: task.task_id });
      } catch (e) {
        const axiosErr = e as AxiosError;
        let errMsg = 'unspecified error';
        if (
          axiosErr.response &&
          typeof axiosErr.response.data.detail === 'string' &&
          axiosErr.response.data.detail.length > 0
        ) {
          errMsg = axiosErr.response.data.detail;
        }
        throw new Error(errMsg);
      }
    },
    [tasksApi],
  );

  return (
    <TaskPanel
      className={classes.taskPanel}
      fetchTasks={fetchTasks}
      cleaningZones={placeNames}
      loopWaypoints={placeNames}
      deliveryWaypoints={placeNames}
      submitTasks={submitTasks}
      cancelTask={cancelTask}
    />
  );
}
