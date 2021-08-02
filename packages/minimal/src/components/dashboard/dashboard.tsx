import {
  Button,
  createStyles,
  makeStyles,
  Grid,
  TablePagination,
  Typography,
  Paper,
} from '@material-ui/core';
import React from 'react';
import type { Task, SubmitTask } from 'api-client';
import { RadioButtonGroup, TaskTable } from 'react-components';
import { DataConfigContext } from '../app-contexts';
import { useAutoRefresh } from './auto-refresh';
import { PlacesContext, RmfIngressContext } from '../rmf-app/contexts';
import { UserContext } from '../auth/contexts';

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
    },
    taskDetails: {
      marginTop: '1em',
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

export default function Dashboard(_props: {}): React.ReactElement {
  const classes = useStyles();
  const data = React.useContext(DataConfigContext);
  const [radioValue, setRadioValue] = React.useState('');
  const user = React.useContext(UserContext);

  const { tasksApi, sioClient } = React.useContext(RmfIngressContext) || {};
  const [autoRefreshState, autoRefreshDispatcher] = useAutoRefresh(sioClient);
  const [task, setTask] = React.useState<SubmitTask>();
  const [page, setPage] = React.useState(0);
  const [hasMore, setHasMore] = React.useState(true);
  const places = React.useContext(PlacesContext);
  const placeNames = places.map((p) => p.vertex.name);
  const paginationOptions: Omit<
    React.ComponentPropsWithoutRef<typeof TablePagination>,
    'component'
  > = {
    page,
    count: hasMore ? -1 : page * 10 + autoRefreshState.tasks.length,
    rowsPerPage: 10,
    rowsPerPageOptions: [10],
    onChangePage: (_ev, newPage) => setPage(newPage),
  };
  const startPlace = placeNames[0];

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
  }, [fetchTasks, page, autoRefreshDispatcher]);

  React.useEffect(() => {
    handleRefresh();
  }, [handleRefresh]);

  function clearForm() {
    setRadioValue('');
  }

  function onHandleChange(value: string) {
    setRadioValue(value);
  }

  React.useEffect(() => {
    const createLoopTask = (destination: string): SubmitTask => {
      return {
        description: {
          start_name: startPlace,
          finish_name: destination,
          num_loops: 1,
        },
        start_time: Math.floor(Date.now() / 1000),
        task_type: 1,
        priority: 0,
      };
    };
    setTask(createLoopTask(radioValue));
  }, [radioValue, startPlace]);

  const submitTask = async () => {
    if (!tasksApi) {
      throw new Error('tasks api not available');
    }
    if (task) {
      await tasksApi.submitTaskTasksSubmitTaskPost(task);
    }
    handleRefresh();
    clearForm();
  };

  return (
    <div className={classes.container}>
      <Paper className={classes.paper}>
        <Grid container>
          <Grid item xs={12}>
            <Typography variant="h6">Loading Bay</Typography>
          </Grid>
          <Grid item xs={12}>
            <RadioButtonGroup
              formLabel={data.radioGroup.formLabel}
              options={placeNames.slice(1)}
              radioGroupName={data.radioGroup.radioGroupTitle}
              onHandleChange={onHandleChange}
            />
          </Grid>
          <Grid item xs={12} className={classes.taskDetails}>
            <Typography variant="subtitle2">Task Details</Typography>
            <Typography variant="body2">Start: {task?.description.start_name}</Typography>
            <Typography variant="body2">End: {task?.description.finish_name}</Typography>
            <Typography variant="body2">Task Type: Loop</Typography>
          </Grid>
          <Grid item xs={12} className={classes.actions}>
            <Button variant="contained" onClick={submitTask}>
              Submit
            </Button>
          </Grid>
        </Grid>
      </Paper>
      {user?.profile.is_admin ? (
        <Paper className={classes.paper}>
          <Grid container>
            <Grid item xs={12} md={8}>
              <TaskTable tasks={autoRefreshState.tasks.map((t) => t.summary)} />
              {paginationOptions && (
                <TablePagination
                  component="div"
                  {...paginationOptions}
                  style={{ flex: '0 0 auto' }}
                />
              )}
            </Grid>
          </Grid>
        </Paper>
      ) : (
        <></>
      )}
    </div>
  );
}
