import {
  Box,
  Button,
  Dialog,
  DialogContent,
  DialogTitle,
  Divider,
  Grid,
  Typography,
} from '@mui/material';
import { CardContent, useTheme } from '@mui/material';
import { TaskEventLog, TaskState } from 'api-client';
import React from 'react';
import { of, switchMap } from 'rxjs';
import { AppControllerContext } from '../app-contexts';
import { AppEvents } from '../app-events';
import { RmfAppContext } from '../rmf-app';
import { TaskInfo } from 'react-components';
import { UserProfileContext } from 'rmf-auth';
import { Enforcer } from '../permissions';
import { TaskLogs } from './task-logs';

export interface TableDataGridState {
  task: TaskState | null;
  open: boolean;
  onClose: () => void;
}

export function TaskInspector({ task, open, onClose }: TableDataGridState): JSX.Element {
  const theme = useTheme();
  const rmf = React.useContext(RmfAppContext);
  const appController = React.useContext(AppControllerContext);

  const [taskState, setTaskState] = React.useState<TaskState | null>(null);
  const [taskLogs, setTaskLogs] = React.useState<TaskEventLog | null>(null);

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
        <Dialog
          open={open}
          onClose={onClose}
          aria-labelledby="alert-dialog-title"
          aria-describedby="alert-dialog-description"
          fullWidth
          maxWidth="md"
        >
          <Grid container spacing={2}>
            <Grid item xs={12}>
              <DialogTitle id="scroll-dialog-title" align="center">
                Task: {task?.booking.id}
              </DialogTitle>
            </Grid>
          </Grid>
          <DialogContent style={{ height: 700 }} dividers={true}>
            <Box sx={{ position: 'relative' }}>
              <Grid container direction="row" wrap="nowrap" height="100%">
                <Grid item xs={6}>
                  {taskState ? (
                    <>
                      <CardContent sx={{ overflow: 'auto' }}>
                        <TaskInfo task={taskState} title="Details" />
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
                    <TaskLogs taskLog={taskLogs} taskState={taskState} title="Logs" />
                  </CardContent>
                </Grid>
              </Grid>
            </Box>
          </DialogContent>
        </Dialog>
      </Grid>
    </>
  );
}
