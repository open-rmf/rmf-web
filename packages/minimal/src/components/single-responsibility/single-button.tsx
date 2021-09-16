import React from 'react';
import { Button, createStyles, makeStyles, Grid, Paper, Snackbar } from '@material-ui/core';
import { Alert, AlertProps } from '@material-ui/lab';
import type { Task, SubmitTask } from 'api-client';
import { DataConfig } from '../../config/data-config';
import { useAutoRefresh } from '../dashboard/auto-refresh';
import { Configuration, SioClient, TasksApi } from 'api-client';
import { AxiosInstance } from 'axios';

interface SingleButtonPageProps {
  data: DataConfig;
  sioClient: SioClient;
  apiConfig: Configuration;
  axiosInst: AxiosInstance;
}

const useStyles = makeStyles((theme) =>
  createStyles({
    container: {
      padding: '2em',
      margin: '2em',
      display: 'flex',
      flexDirection: 'row',
      [theme.breakpoints.down('md')]: {
        display: 'flex',
        flexDirection: 'column',
        flexWrap: 'nowrap',
      },
      textAlign: 'center',
      justifyContent: 'space-around',
    },
    paper: {
      padding: '2em',
      marginLeft: '0.5em',
      marginRight: '0.5em',
      [theme.breakpoints.down('md')]: {
        marginBottom: '1em',
      },
    },
    actions: {
      marginTop: '1em',
      textAlign: 'right',
    },
  }),
);

export default function SingleButtonPage(props: SingleButtonPageProps) {
  const classes = useStyles();
  const { data, apiConfig, axiosInst, sioClient } = props;
  const [snackbarSeverity, setSnackbarSeverity] = React.useState<AlertProps['severity']>('success');
  const [openSnackbar, setOpenSnackbar] = React.useState(false);
  const [snackbarMessage, setSnackbarMessage] = React.useState('');

  const [autoRefreshState, autoRefreshDispatcher] = useAutoRefresh(sioClient);
  const [task, setTask] = React.useState<SubmitTask>();
  const [page, setPage] = React.useState(0);
  const [hasMore, setHasMore] = React.useState(true);

  const tasksApi = React.useMemo(() => {
    return new TasksApi(apiConfig, undefined, axiosInst);
  }, [apiConfig, axiosInst]);

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
        10,
        page * 10,
        '-priority,-start_time',
      );
      const tasks = resp.data as Task[];
      setHasMore(tasks.length > 10);
      return tasks.slice(0, 10);
    },
    [tasksApi],
  );

  const handleRefresh = React.useCallback(async () => {
    autoRefreshDispatcher.setTasks(await fetchTasks(page));
  }, [autoRefreshDispatcher, fetchTasks, page]);

  React.useEffect(() => {
    const createLoopTask = (): SubmitTask => {
      return {
        description: {
          start_name: data.loopTaskDetails.start,
          finish_name: 'pantry',
          num_loops: 1,
        },
        start_time: Math.floor(Date.now() / 1000),
        task_type: 1,
        priority: 0,
      };
    };
    setTask(createLoopTask());
  }, [data]);

  const submitTask = async () => {
    if (!tasksApi) {
      setSnackbarSeverity('error');
      setOpenSnackbar(true);
      setSnackbarMessage('Tasks api not available');
    }
    if (task) {
      await tasksApi.submitTaskTasksSubmitTaskPost(task);
      setSnackbarSeverity('success');
      setOpenSnackbar(true);
      setSnackbarMessage('Task submitted successfully');
    }
    handleRefresh();
  };

  return (
    <div className={classes.container}>
      <Paper className={classes.paper}>
        <Grid container>
          <Grid item xs={12} className={classes.actions}>
            <Button variant="contained" onClick={submitTask}>
              Submit
            </Button>
          </Grid>
        </Grid>
      </Paper>
      <Snackbar open={openSnackbar} onClose={() => setOpenSnackbar(false)} autoHideDuration={2000}>
        <Alert severity={snackbarSeverity}>{snackbarMessage}</Alert>
      </Snackbar>
    </div>
  );
}
