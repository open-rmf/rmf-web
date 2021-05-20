import { makeStyles } from '@material-ui/core';
import type { TaskProgress } from 'api-client';
import React from 'react';
import { TaskPanel, TaskPanelProps } from 'react-components';
import { PlacesContext, RmfIngressContext } from '../rmf-app';

const useStyles = makeStyles((theme) => ({
  taskPanel: {
    padding: `${theme.spacing(4)}px`,
    height: '100%',
    backgroundColor: theme.palette.secondary.main,
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

  const submitTask = React.useCallback<Required<TaskPanelProps>['submitTask']>(
    async (body) => {
      await tasksApi?.submitTaskTasksSubmitTaskPost(body);
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
      submitTask={submitTask}
    />
  );
}
