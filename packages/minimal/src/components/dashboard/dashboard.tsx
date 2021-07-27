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
import * as RmfModels from 'rmf-models';
import { RadioButtonGroup, TaskTable } from 'react-components';
import { DataConfigContext } from '../app-contexts';
import { useAutoRefresh } from './auto-refresh';
import { PlacesContext, RmfIngressContext } from '../rmf-app/contexts';
import { SubmitTask, TaskProgress } from '../../../../api-client/dist';

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
    paper: {
      padding: '2em',
      marginLeft: '0.5em',
      marginRight: '0.5em',
      [theme.breakpoints.down('md')]: {
        marginBottom: '1em',
      },
    },
    actions: {
      textAlign: 'right',
    },
  }),
);

export default function Dashboard(_props: {}): React.ReactElement {
  const classes = useStyles();
  const data = React.useContext(DataConfigContext);
  const [radioValue, setRadioValue] = React.useState('');
  const appUser = process.env.REACT_APP_USER;

  const { tasksApi, sioClient } = React.useContext(RmfIngressContext) || {};
  const [task, setTask] = React.useState<SubmitTask>();
  const [autoRefreshState, autoRefreshDispatcher] = useAutoRefresh(sioClient);
  const [page, setPage] = React.useState(0);
  const [totalCount, setTotalCount] = React.useState(0);
  const places = React.useContext(PlacesContext);
  const placeNames = places.map((p) => p.vertex.name);
  const paginationOptions: Omit<
    React.ComponentPropsWithoutRef<typeof TablePagination>,
    'component'
  > = {
    page,
    count: totalCount,
    rowsPerPage: 10,
    rowsPerPageOptions: [10],
    onChangePage: (_ev, newPage) => setPage(newPage),
  };

  const createLoopTask = (destination: string) => {
    return {
      description: {
        start_name: 'supplies',
        finish_name: destination,
        num_loops: 1,
      },
      start_time: Math.floor(Date.now() / 1000),
      task_type: 1,
      priority: 0,
    };
  };

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
    setTask(createLoopTask(radioValue));
  }, [radioValue]);

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
              options={placeNames}
              radioGroupName={data.radioGroup.radioGroupTitle}
              row={true}
              onHandleChange={onHandleChange}
            />
          </Grid>
          <Grid item xs={12}>
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
      {appUser === 'admin' ? (
        <Paper className={classes.paper}>
          <Grid container>
            <Grid item xs={12}>
              <TaskTable tasks={autoRefreshState.tasks} />
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
