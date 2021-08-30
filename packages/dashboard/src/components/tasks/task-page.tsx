/* istanbul ignore file */

import { makeStyles } from '@material-ui/core';
import type { Task, TaskSummary } from 'api-client';
import type { AxiosError } from 'axios';
import React from 'react';
import { PlacesContext, RmfIngressContext } from '../rmf-app';
import { TaskPanel, TaskPanelProps } from './task-panel';

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
  const { tasksApi, sioClient } = React.useContext(RmfIngressContext) || {};
  const [fetchedTasks, setFetchedTasks] = React.useState<Task[]>([]);
  const [updatedSummaries, setUpdatedSummaries] = React.useState<Record<string, TaskSummary>>({});
  const [autoRefreshEnabled, setAutoRefreshEnabled] = React.useState(true);
  const [page, setPage] = React.useState(0);
  const [hasMore, setHasMore] = React.useState(true);
  const places = React.useContext(PlacesContext);
  const placeNames = places.map((p) => p.vertex.name);
  const tasks = React.useMemo(
    () => fetchedTasks.map((t) => ({ ...t, summary: updatedSummaries[t.task_id] || t.summary })),
    [fetchedTasks, updatedSummaries],
  );

  const fetchTasks = React.useCallback(
    async (page: number) => {
      if (!tasksApi) {
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
        11,
        page * 10,
        '-priority,-start_time',
      );
      const results = resp.data as Task[];
      setHasMore(results.length > 10);
      setFetchedTasks(results.slice(0, 10));
    },
    [tasksApi],
  );

  React.useEffect(() => {
    if (!autoRefreshEnabled || !sioClient) return;
    const subs = fetchedTasks.map((t) =>
      sioClient.subscribeTaskSummary(t.task_id, (newSummary) =>
        setUpdatedSummaries((prev) => ({
          ...prev,
          [newSummary.task_id]: newSummary,
        })),
      ),
    );
    return () => {
      subs.forEach((s) => sioClient.unsubscribe(s));
    };
  }, [autoRefreshEnabled, sioClient, fetchedTasks]);

  const handleRefresh = React.useCallback<Required<TaskPanelProps>['onRefresh']>(async () => {
    fetchTasks(page);
  }, [fetchTasks, page]);

  React.useEffect(() => {
    handleRefresh();
  }, [handleRefresh]);

  const submitTasks = React.useCallback<Required<TaskPanelProps>['submitTasks']>(
    async (tasks) => {
      if (!tasksApi) {
        throw new Error('tasks api not available');
      }
      await Promise.all(tasks.map((t) => tasksApi.submitTaskTasksSubmitTaskPost(t)));
      handleRefresh();
    },
    [tasksApi, handleRefresh],
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
        count: hasMore ? -1 : page * 10 + tasks.length,
        rowsPerPage: 10,
        rowsPerPageOptions: [10],
        onChangePage: (_ev, newPage) => setPage(newPage),
      }}
      cleaningZones={placeNames}
      loopWaypoints={placeNames}
      deliveryWaypoints={placeNames}
      submitTasks={submitTasks}
      cancelTask={cancelTask}
      onRefresh={handleRefresh}
      onAutoRefresh={setAutoRefreshEnabled}
      timeline={true}
    />
  );
}
