import React from 'react';
import * as RmfModels from 'rmf-models';
import type { TaskProgress } from 'api-client';
import type { AxiosError } from 'axios';
import { PlacesContext, RmfIngressContext } from '../rmf-app';
import { CustomForm, CustomTaskPanel, TaskPanelProps } from 'react-components';
import { Grid, makeStyles, Paper, Typography } from '@material-ui/core';
import { useAutoRefresh } from '../tasks/auto-refresh';

const useStyles = makeStyles((theme) => ({
  paper: {
    padding: theme.spacing(2),
    margin: `${theme.spacing(2)}px`,
    textAlign: 'center',
  },
  taskPanel: {
    height: '100%',
    maxWidth: 1600,
  },
  formContainer: {
    margin: `auto ${theme.spacing(2)}px`,
    padding: `${theme.spacing(2)}px`,
    height: '100%',
  },
  tasksPanelContainer: {
    margin: `auto ${theme.spacing(2)}px`,
    marginLeft: 0,
    padding: `${theme.spacing(2)}px`,
    height: '100%',
  },
}));

const BeverageStationPage = () => {
  const classes = useStyles();
  const { fleetsApi, tasksApi, sioClient } = React.useContext(RmfIngressContext) || {};
  const [autoRefreshState, autoRefreshDispatcher] = useAutoRefresh(sioClient);

  const [fleets, setFleets] = React.useState<[]>([]);
  const [page, setPage] = React.useState(0);
  const [totalCount, setTotalCount] = React.useState(0);
  const [tasks, setTasks] = React.useState<RmfModels.TaskSummary[]>([]);
  const [selectedTask, setSelectedTask] = React.useState<RmfModels.TaskSummary | undefined>(
    undefined,
  );
  const places = React.useContext(PlacesContext);
  const placeNames = places.map((p) => p.vertex.name);

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
      return taskProgresses.map((t) => t.task_summary) as RmfModels.TaskSummary[];
    },
    [tasksApi],
  );

  const handleRefresh = React.useCallback<Required<TaskPanelProps>['onRefresh']>(async () => {
    autoRefreshDispatcher.setTasks(await fetchTasks(page));
  }, [fetchTasks, page, autoRefreshDispatcher]);

  React.useEffect(() => {
    handleRefresh();
  }, [handleRefresh]);

  const submitTasks = React.useCallback(async (tasks) => {
    // if (!tasksApi) {
    //   throw new Error('tasks api not available');
    // }
    // await Promise.all(tasks.map((t) => tasksApi.submitTaskTasksSubmitTaskPost(t)));
    // handleRefresh();
  }, []);

  const cancelTask = React.useCallback(
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

  const fetchFleets = React.useCallback(async () => {
    if (!fleetsApi) {
      return [];
    }
    const resp = await fleetsApi?.getFleetsFleetsGet();
    setFleets(resp.data.items);
    return resp.data.items;
  }, [fleetsApi]);

  React.useEffect(() => {
    fetchFleets();
  }, [fetchFleets]);

  return (
    <Grid container>
      <Grid item xs={12}>
        <Paper className={classes.paper}>
          <Typography variant="h5">Beverage Station</Typography>
        </Paper>
      </Grid>
      <Grid item xs={2}>
        <Paper className={classes.formContainer}>
          <Typography variant="h6">Task Creator</Typography>
          <CustomForm
            deliveryWaypoints={[]}
            dispensers={[]}
            ingestors={[]}
            submitTasks={submitTasks}
          />
        </Paper>
      </Grid>
      <Grid item xs={10}>
        <Paper className={classes.tasksPanelContainer}>
          <CustomTaskPanel
            className={classes.taskPanel}
            tasks={autoRefreshState.tasks}
            paginationOptions={{
              page,
              count: totalCount,
              rowsPerPage: 10,
              rowsPerPageOptions: [10],
              onChangePage: (_ev, newPage) => setPage(newPage),
            }}
            onAutoRefresh={autoRefreshDispatcher.setEnabled}
          />
        </Paper>
      </Grid>
    </Grid>
  );
};

export default BeverageStationPage;
