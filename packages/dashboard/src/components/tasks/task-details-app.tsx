import { Button, CardContent, Grid, Typography, useTheme } from '@mui/material';
import { TaskStateOutput as TaskState } from 'api-client';
import React from 'react';
import { TaskInfo } from 'react-components';
// import { UserProfileContext } from 'rmf-auth';
import { of, switchMap } from 'rxjs';

import { useAppController } from '../../hooks/use-app-controller';
import { useRmfApi } from '../../hooks/use-rmf-api';
import { AppEvents } from '../app-events';
// import { Enforcer } from '../permissions';

export const TaskDetailsCard = () => {
  const theme = useTheme();
  const rmfApi = useRmfApi();
  const appController = useAppController();

  const [taskState, setTaskState] = React.useState<TaskState | null>(null);
  React.useEffect(() => {
    const sub = AppEvents.taskSelect
      .pipe(
        switchMap((selectedTask) =>
          selectedTask ? rmfApi.getTaskStateObs(selectedTask.booking.id) : of(null),
        ),
      )
      .subscribe(setTaskState);
    return () => sub.unsubscribe();
  }, [rmfApi]);

  // const profile = React.useContext(UserProfileContext);
  const taskCancellable =
    taskState &&
    // profile &&
    // Enforcer.canCancelTask(profile) &&
    taskState.status &&
    !['canceled', 'killed', 'completed', 'failed'].includes(taskState.status);
  const handleCancelTaskClick = React.useCallback<React.MouseEventHandler>(async () => {
    if (!taskState) {
      return;
    }
    try {
      await rmfApi.tasksApi?.postCancelTaskTasksCancelTaskPost({
        type: 'cancel_task_request',
        task_id: taskState.booking.id,
      });
      appController.showAlert('success', 'Successfully cancelled task');
      AppEvents.taskSelect.next(null);
    } catch (e) {
      appController.showAlert('error', `Failed to cancel task: ${(e as Error).message}`);
    }
  }, [appController, taskState, rmfApi]);

  return (
    <Grid container direction="column" wrap="nowrap" height="100%">
      {taskState ? (
        <>
          <CardContent sx={{ overflow: 'auto' }}>
            <TaskInfo task={taskState} />
          </CardContent>
          <Grid item paddingLeft={2} paddingRight={2}>
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
  );
};

export default TaskDetailsCard;
