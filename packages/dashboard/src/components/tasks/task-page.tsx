/* istanbul ignore file */

import { makeStyles } from '@material-ui/core';
import type { TaskProgress } from 'api-client';
import type { AxiosError } from 'axios';
import React from 'react';
import { TaskPanel, TaskPanelProps } from 'react-components';
import * as RmfModels from 'rmf-models';
import { PlacesContext, RmfIngressContext } from '../rmf-app';
import { useAutoRefresh } from './auto-refresh';
import { sortTasks } from './utils';

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
  const [autoRefreshState, autoRefreshDispatcher] = useAutoRefresh(sioClient);
  const [page, setPage] = React.useState(0);
  const [totalCount, setTotalCount] = React.useState(0);
  const places = React.useContext(PlacesContext);

  const cleanZonesTemp = Object.values(places).reduce<string[]>((place, it) => {
    for (const param of it.vertex.params) {
      if (param.name === 'is_cleaning_zone' && param.value_bool) {
        place.push(it.vertex.name);
        break;
      }
    }
    return place;
  }, []);
  const cleanZones = [...Array.from(new Set(cleanZonesTemp))];

  // TODO should retain workcell name, and parse it to TaskPanel
  const deliveryPlacesTemp = Object.values(places).reduce<string[]>((place, it) => {
    const param_names = it.vertex.params.map((param) => param.name);
    if (param_names.includes('pickup_dispenser') || param_names.includes('dropoff_ingestor'))
      place.push(it.vertex.name);
    return place;
  }, []);
  const deliveryPlaces = [...Array.from(new Set(deliveryPlacesTemp))];

  // TODO This is custom to the throwaway demo. We are not showing any
  // charging waypoints and cleaning waypoints on loop request. dock_name is valid
  // for charging and cleaning waypoints
  const loopPlacesTemp = Object.values(places).reduce<string[]>((place, it) => {
    const param_names = it.vertex.params.map((param) => param.name);
    if (!param_names.includes('dock_name')) place.push(it.vertex.name);
    return place;
  }, []);
  const loopPlaces = [...Array.from(new Set(loopPlacesTemp))];

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
        20,
        page * 20,
        'state,start_time,-priority',
      );
      setTotalCount(resp.data.total_count);
      const taskProgresses: TaskProgress[] = resp.data.items;
      return sortTasks(taskProgresses.map((t) => t.task_summary)) as RmfModels.TaskSummary[];
    },
    [tasksApi],
  );

  const handleRefresh = React.useCallback<Required<TaskPanelProps>['onRefresh']>(async () => {
    autoRefreshDispatcher.setTasks(await fetchTasks(page));
  }, [fetchTasks, page, autoRefreshDispatcher]);

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
        handleRefresh();
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
    [tasksApi, handleRefresh],
  );

  return (
    <TaskPanel
      className={classes.taskPanel}
      tasks={sortTasks(autoRefreshState.tasks)}
      paginationOptions={{
        page,
        count: totalCount,
        rowsPerPage: 20,
        rowsPerPageOptions: [20],
        onChangePage: (_ev, newPage) => setPage(newPage),
      }}
      cleaningZones={cleanZones}
      loopWaypoints={loopPlaces}
      deliveryWaypoints={deliveryPlaces}
      submitTasks={submitTasks}
      cancelTask={cancelTask}
      onRefresh={handleRefresh}
      onAutoRefresh={autoRefreshDispatcher.setEnabled}
    />
  );
}
