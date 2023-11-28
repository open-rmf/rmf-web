import { Button, ButtonProps, Theme, Tooltip } from '@mui/material';
import { TaskState } from 'api-client';
import React from 'react';
import { AppControllerContext } from '../app-contexts';
import { AppEvents } from '../app-events';
import { RmfAppContext } from '../rmf-app';
import { UserProfileContext } from 'rmf-auth';
import { Enforcer } from '../permissions';
import { makeStyles, createStyles } from '@mui/styles';

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    enableHover: {
      '&.Mui-disabled': {
        pointerEvents: 'auto',
      },
    },
  }),
);

export interface TaskCancelButtonProp extends ButtonProps {
  taskId: string | null;
}

export function TaskCancelButton({ taskId, ...otherProps }: TaskCancelButtonProp): JSX.Element {
  const classes = useStyles();
  const rmf = React.useContext(RmfAppContext);
  const appController = React.useContext(AppControllerContext);
  const profile = React.useContext(UserProfileContext);

  const [taskState, setTaskState] = React.useState<TaskState | null>(null);

  React.useEffect(() => {
    if (!rmf || !taskId) {
      return;
    }
    const sub = rmf.getTaskStateObs(taskId).subscribe(setTaskState);
    return () => sub.unsubscribe();
  }, [rmf, taskId]);

  const taskCancellable =
    taskState &&
    taskState.status &&
    !['canceled', 'killed', 'completed', 'failed'].includes(taskState.status);
  const userCanCancelTask = profile && Enforcer.canCancelTask(profile);

  function capitalizeFirstLetter(status: string): string {
    return status.charAt(0).toUpperCase() + status.slice(1);
  }

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

  return taskCancellable && userCanCancelTask ? (
    <Button onClick={handleCancelTaskClick} autoFocus {...otherProps}>
      Cancel Task
    </Button>
  ) : taskCancellable && !userCanCancelTask ? (
    <Tooltip title="You don't have permission to cancel tasks.">
      <Button disabled className={classes['enableHover']} {...otherProps}>
        Cancel Task
      </Button>
    </Tooltip>
  ) : (
    <Tooltip
      title={
        taskState && taskState.status
          ? `${capitalizeFirstLetter(taskState.status)} task cannot be cancelled.`
          : `Task cannot be cancelled.`
      }
    >
      <Button disabled className={classes['enableHover']} {...otherProps}>
        Cancel Task
      </Button>
    </Tooltip>
  );
}
