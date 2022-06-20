import { Button, CardContent, Grid, Typography, useTheme } from '@mui/material';
import { TaskState } from 'api-client';
import React from 'react';
import { TaskInfo } from 'react-components';
import { UserProfileContext } from 'rmf-auth';
import { of, switchMap } from 'rxjs';
import { AppControllerContext } from '../app-contexts';
import { AppEvents } from '../app-events';
import { createMicroApp } from '../micro-app';
import { Enforcer } from '../permissions';
import { RmfAppContext } from '../rmf-app';

export const TaskDetailsApp = createMicroApp('Task Details', () => {
  const theme = useTheme();
  const rmf = React.useContext(RmfAppContext);
  const appController = React.useContext(AppControllerContext);

  const [taskState, setTaskState] = React.useState<TaskState | null>(null);
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

  const profile = React.useContext(UserProfileContext);
  const taskCancellable =
    taskState &&
    profile &&
    Enforcer.canCancelTask(profile) &&
    (taskState.active || taskState.pending);
  const handleCancelTaskClick = React.useCallback<React.MouseEventHandler>(async () => {
    if (!taskState) {
      return;
    }
    try {
      appController.showAlert('success', 'Successfully cancelled task');
      AppEvents.taskSelect.next(null);
    } catch (e) {
      appController.showAlert('error', `Failed to cancel task: ${(e as Error).message}`);
    }
  }, [appController, taskState]);

  return (
    <CardContent sx={{ height: '100%', boxSizing: 'border-box' }}>
      {taskState ? (
        <>
          <TaskInfo task={taskState} />
          <Button
            style={{ marginTop: theme.spacing(1), marginBottom: theme.spacing(1) }}
            fullWidth
            variant="contained"
            color="secondary"
            aria-label="Cancel Task"
            disabled={!taskCancellable}
            onClick={handleCancelTaskClick}
          >
            Cancel Task
          </Button>
        </>
      ) : (
        <Grid container wrap="nowrap" alignItems="center" style={{ height: '100%' }}>
          <Typography variant="h6" align="center">
            Click on a task to view more information
          </Typography>
        </Grid>
      )}
    </CardContent>
  );
});
