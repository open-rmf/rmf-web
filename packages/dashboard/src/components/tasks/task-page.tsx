/* istanbul ignore file */

import { AppBar, Box, makeStyles, Tab, Tabs, Typography } from '@material-ui/core';
import type { Task } from 'api-client';
import type { AxiosError } from 'axios';
import React from 'react';
import { getScheduledTasksAPI, getTaskRulesAPI } from '../../managers/task-scheduler-manager';
import { PlacesContext, RmfIngressContext } from '../rmf-app';
import { useAutoRefresh } from './auto-refresh';
import { TabPanel, a11yProps } from './tab-panel';
import { TaskPanel, TaskPanelProps } from './task-panel';
import TaskSchedulerPanel, { TaskSchedulerPanelProps } from './task-schedule-panel';

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
  const [hasMore, setHasMore] = React.useState(true);
  const places = React.useContext(PlacesContext);
  const placeNames = places.map((p) => p.vertex.name);

  const [value, setValue] = React.useState(0);

  const handleChange = (event: any, newValue: number) => {
    setValue(newValue);
  };

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
      const tasks = resp.data as Task[];
      setHasMore(tasks.length > 10);
      return tasks.slice(0, 10);
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

  /*
  ---------------
  */

  const submitTaskSchedule = React.useCallback<Required<TaskSchedulerPanelProps>['submitTask']>(
    async (task) => {
      // await Promise.all(tasks.map((t) => tasksApi.submitTaskTasksSubmitTaskPost(t)));
      // handleRefresh();
    },
    [],
  );

  const getTaskRules = React.useCallback(async (offset: number) => {
    console.log(offset);
    const response = await getTaskRulesAPI(0);
    console.log(response);
    return [];
    // await Promise.all(tasks.map((t) => tasksApi.submitTaskTasksSubmitTaskPost(t)));
    // handleRefresh();
  }, []);

  const getScheduledTasks = React.useCallback(async (offset: number) => {
    console.log(offset);
    const response = await getScheduledTasksAPI(0);
    console.log(response);
    // await Promise.all(tasks.map((t) => tasksApi.submitTaskTasksSubmitTaskPost(t)));
    // handleRefresh();
    return [];
  }, []);

  return (
    <>
      <AppBar position="static">
        <Tabs value={value} onChange={handleChange} aria-label="simple tabs example">
          <Tab label="Tasks" {...a11yProps(0)} />
          <Tab label="Schedule Task" {...a11yProps(1)} />
        </Tabs>
      </AppBar>
      <TabPanel value={value} index={0}>
        <TaskPanel
          className={classes.taskPanel}
          tasks={autoRefreshState.tasks}
          paginationOptions={{
            page,
            count: hasMore ? -1 : page * 10 + autoRefreshState.tasks.length,
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
          onAutoRefresh={autoRefreshDispatcher.setEnabled}
        />
      </TabPanel>
      <TabPanel value={value} index={1}>
        <TaskSchedulerPanel
          className={classes.taskPanel}
          tasks={autoRefreshState.tasks}
          getTaskRules={getTaskRules}
          getScheduledTasks={getScheduledTasks}
          paginationOptions={{
            page,
            count: hasMore ? -1 : page * 10 + autoRefreshState.tasks.length,
            rowsPerPage: 10,
            rowsPerPageOptions: [10],
            onChangePage: (_ev, newPage) => setPage(newPage),
          }}
          cleaningZones={placeNames}
          loopWaypoints={placeNames}
          deliveryWaypoints={placeNames}
          submitTask={submitTaskSchedule}
          cancelTask={cancelTask}
        ></TaskSchedulerPanel>
      </TabPanel>
    </>
  );
}
