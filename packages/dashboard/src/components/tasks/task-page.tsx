/* istanbul ignore file */

import { makeStyles } from '@material-ui/core';
import type { TaskProgress } from 'api-client';
import type { AxiosError } from 'axios';
import React from 'react';
import { TaskPanel, TaskPanelProps } from 'react-components';
import * as RmfModels from 'rmf-models';
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
  const [tasks, setTasks] = React.useState<RmfModels.TaskSummary[]>([]);
  const [page, setPage] = React.useState(0);
  const [totalCount, setTotalCount] = React.useState(0);
  const { tasksApi = null } = React.useContext(RmfIngressContext) || {};
  const places = React.useContext(PlacesContext);

  const fetchTasks = React.useCallback(
    async (page: number) => {
      if (!tasksApi) {
        setTotalCount(0);
        return [];
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
        10,
        page * 10,
        '-priority,-start_time',
      );
      setTotalCount(resp.data.total_count);
      const taskProgresses: TaskProgress[] = resp.data.items;
      return taskProgresses.map((t) => t.task_summary);
    },
    [tasksApi],
  );

  React.useEffect(() => {
    (async () => {
      setTasks(await fetchTasks(page));
    })();
  }, [fetchTasks, page]);

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
      tasks={tasks}
      paginationOptions={{
        page,
        count: totalCount,
        rowsPerPage: 10,
        rowsPerPageOptions: [10],
        onChangePage: (_ev, newPage) => setPage(newPage),
      }}
      cleaningZones={Object.keys(places)}
      loopWaypoints={Object.keys(places)}
      deliveryWaypoints={Object.keys(places)}
      submitTasks={submitTasks}
      cancelTask={cancelTask}
    />
  );
}
