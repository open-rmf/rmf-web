import {
  Box,
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogContentText,
  DialogProps,
  DialogTitle,
  Divider,
  Grid,
  List,
  Typography,
} from '@mui/material';
import { CardContent, useTheme } from '@mui/material';
import { TaskEventLog, TaskState } from 'api-client';
import React from 'react';
import { of, switchMap } from 'rxjs';
import { AppControllerContext } from '../app-contexts';
import { AppEvents } from '../app-events';
import { AppRegistry } from '../app-registry';
import { MicroAppProps } from '../micro-app';
import { RmfAppContext } from '../rmf-app';
import { WorkspaceState } from '../workspace';
import { TestDetailMicroTask } from './test-details';
import { TaskInfo } from 'react-components';
import { UserProfileContext } from 'rmf-auth';
import { Enforcer } from '../permissions';
import { TaskLogs } from './task-logs';

export interface TableDataGridState {
  task: TaskState | null;
  open: boolean;
  onClose: (test: boolean) => void;
}

export function TaskLogsDetails({ task, open, onClose }: TableDataGridState): JSX.Element {
  const theme = useTheme();
  const rmf = React.useContext(RmfAppContext);
  const appController = React.useContext(AppControllerContext);

  const [taskState, setTaskState] = React.useState<TaskState | null>(null);
  const [taskLogs, setTaskLogs] = React.useState<TaskEventLog | null>(null);
  const [scroll, setScroll] = React.useState<DialogProps['scroll']>('paper');

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = AppEvents.taskSelect.subscribe((task) => {
      if (!task) {
        setTaskState(null);
        setTaskLogs(null);
        return;
      }
      (async () => {
        // TODO: Get full logs, then subscribe to log updates for new logs.
        // Unlike with state events, we can't just subscribe to logs updates.
        try {
          const logs = (
            await rmf.tasksApi.getTaskLogTasksTaskIdLogGet(
              task.booking.id,
              `0,${Number.MAX_SAFE_INTEGER}`,
            )
          ).data;
          setTaskLogs(logs);
        } catch {
          console.log(`Failed to fetch task logs for ${task.booking.id}`);
          setTaskLogs(null);
        }
        setTaskState(task);
      })();
    });
    return () => sub.unsubscribe();
  }, [rmf]);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = AppEvents.taskSelect
      .pipe(
        switchMap((selectedTask) =>
          selectedTask ? rmf.getTaskStateObs(selectedTask.booking.id) : of(null),
        ),
      )
      .subscribe(setTaskState);
    return () => sub.unsubscribe();
  }, [rmf]);

  React.useEffect(() => {
    AppEvents.taskSelect.next(task);
  }, [task]);

  console.log(task);

  const profile = React.useContext(UserProfileContext);
  const taskCancellable =
    taskState &&
    profile &&
    Enforcer.canCancelTask(profile) &&
    taskState.status &&
    !['canceled', 'killed', 'completed', 'failed'].includes(taskState.status);
  const handleCancelTaskClick = React.useCallback<React.MouseEventHandler>(async () => {
    if (!taskState) {
      return;
    }
    try {
      if (!rmf) {
        throw new Error('tasks api not available');
      }
      await rmf.tasksApi?.postCancelTaskTasksCancelTaskPost({
        type: 'cancel_task_request',
        task_id: taskState.booking.id,
      });
      appController.showAlert('success', 'Successfully cancelled task');
      AppEvents.taskSelect.next(null);
    } catch (e) {
      appController.showAlert('error', `Failed to cancel task: ${(e as Error).message}`);
    }
  }, [appController, taskState, rmf]);

  return (
    <>
      <Grid container wrap="nowrap" direction="column" height="100%">
        <div>
          <Dialog
            open={open}
            onClose={() => onClose(false)}
            aria-labelledby="alert-dialog-title"
            aria-describedby="alert-dialog-description"
            fullWidth
            maxWidth="md"
          >
            <Grid container spacing={2}>
              <Grid item xs={6}>
                <DialogTitle id="scroll-dialog-title">
                  <Typography variant="h5" align="center">
                    Details
                  </Typography>
                  <Typography>Details</Typography>
                  <Typography>Details</Typography>
                  <Typography>Details</Typography>
                </DialogTitle>
              </Grid>
              <Grid item xs={6} alignContent="center" justifyContent="center">
                <DialogTitle id="scroll-dialog-title" sx={{ textAlign: 'center' }}>
                  Logs
                </DialogTitle>
              </Grid>
            </Grid>
            <DialogContent style={{ height: 700 }} dividers={scroll === 'paper'}>
              <Box sx={{ position: 'relative' }}>
                <Grid container direction="row" wrap="nowrap" height="100%">
                  <Grid item xs={6}>
                    {taskState ? (
                      <>
                        <CardContent sx={{ overflow: 'auto' }}>
                          <TaskInfo task={taskState} />
                        </CardContent>
                        <Grid item paddingLeft={2} paddingRight={2}>
                          <Button
                            style={{
                              marginTop: theme.spacing(1),
                              marginBottom: theme.spacing(1),
                            }}
                            fullWidth
                            variant="contained"
                            color="secondary"
                            aria-label="Cancel Task"
                            disabled={!taskCancellable}
                            onClick={handleCancelTaskClick}
                          >
                            Cancel Task
                          </Button>
                        </Grid>
                      </>
                    ) : (
                      <Grid container wrap="nowrap" alignItems="center" style={{ height: '100%' }}>
                        <CardContent>
                          <Typography variant="h6" align="center">
                            Click on a task to view more information
                          </Typography>
                        </CardContent>
                      </Grid>
                    )}
                  </Grid>
                  <Divider
                    orientation="vertical"
                    flexItem
                    style={{ marginLeft: theme.spacing(2), marginRight: theme.spacing(2) }}
                  />
                  <Grid item xs={6}>
                    <CardContent>
                      <TaskLogs taskLog={taskLogs} taskState={taskState} />
                    </CardContent>
                  </Grid>
                </Grid>

                {/* <TestDetailMicroTask task={task} /> */}
                {/* {workspaceState.windows.map((w) => {
                  const MicroApp = AppRegistry[w.appName] || null;
                  return MicroApp ? (
                    <MicroApp
                      key={w.key}
                      onClose={() => {
                        setWorkspaceState &&
                          setWorkspaceState({
                            layout: workspaceState.layout.filter((l) => l.i !== w.key),
                            windows: workspaceState.windows.filter((w2) => w2.key !== w.key),
                          });
                      }}
                    />
                  ) : (
                    <div></div>
                  );
                })} */}
              </Box>
            </DialogContent>
          </Dialog>
        </div>
      </Grid>
    </>
  );
}
