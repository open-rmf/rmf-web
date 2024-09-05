import { Button, ButtonProps, Tooltip, Typography } from '@mui/material';
import { TaskStateOutput as TaskState } from 'api-client';
import React from 'react';
import { ConfirmationDialog } from 'react-components';

import { useAppController } from '../../hooks/use-app-controller';
import { useRmfApi } from '../../hooks/use-rmf-api';
import { useUserProfile } from '../../hooks/use-user-profile';
import { Enforcer } from '../../services/permissions';
import { AppEvents } from '../app-events';

export interface TaskCancelButtonProp extends ButtonProps {
  taskId: string | null;
  buttonText?: string;
}

export function TaskCancelButton({
  taskId,
  buttonText,
  ...otherProps
}: TaskCancelButtonProp): JSX.Element {
  const rmfApi = useRmfApi();
  const appController = useAppController();
  const profile = useUserProfile();

  const [taskState, setTaskState] = React.useState<TaskState | null>(null);
  const [openConfirmDialog, setOpenConfirmDialog] = React.useState(false);

  React.useEffect(() => {
    if (!taskId) {
      return;
    }
    const sub = rmfApi.getTaskStateObs(taskId).subscribe((state) => {
      setTaskState(state);
    });
    return () => sub.unsubscribe();
  }, [rmfApi, taskId]);

  const isTaskCancellable = (state: TaskState | null) => {
    return (
      state && state.status && !['canceled', 'killed', 'completed', 'failed'].includes(state.status)
    );
  };

  const userCanCancelTask = profile && Enforcer.canCancelTask(profile);

  function capitalizeFirstLetter(status: string): string {
    return status.charAt(0).toUpperCase() + status.slice(1);
  }

  const handleCancelTaskClick = React.useCallback<React.MouseEventHandler>(async () => {
    if (!taskState) {
      return;
    }
    try {
      await rmfApi.tasksApi?.postCancelTaskTasksCancelTaskPost({
        type: 'cancel_task_request',
        task_id: taskState.booking.id,
        labels: profile ? [profile.user.username] : undefined,
      });
      appController.showAlert('success', 'Task cancellation requested');
      AppEvents.taskSelect.next(null);
      AppEvents.refreshTaskApp.next();
    } catch (e) {
      appController.showAlert('error', `Failed to cancel task: ${(e as Error).message}`);
    }
    setOpenConfirmDialog(false);
  }, [appController, taskState, rmfApi, profile]);

  return (
    <>
      {isTaskCancellable(taskState) && userCanCancelTask ? (
        <Button onClick={() => setOpenConfirmDialog(true)} autoFocus {...otherProps}>
          {buttonText ?? 'Cancel Task'}
        </Button>
      ) : isTaskCancellable(taskState) && !userCanCancelTask ? (
        <Tooltip title="You don't have permission to cancel tasks.">
          <Button
            disabled
            sx={{
              '&.Mui-disabled': {
                pointerEvents: 'auto',
              },
            }}
            {...otherProps}
          >
            {buttonText ?? 'Cancel Task'}
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
          <Button
            disabled
            sx={{
              '&.Mui-disabled': {
                pointerEvents: 'auto',
              },
            }}
            {...otherProps}
          >
            {buttonText ?? 'Cancel Task'}
          </Button>
        </Tooltip>
      )}
      {openConfirmDialog && (
        <ConfirmationDialog
          confirmText="Confirm"
          cancelText="Cancel"
          open={openConfirmDialog}
          title={`Cancel task [${taskState?.booking.id || 'n/a'}]`}
          submitting={undefined}
          onClose={() => {
            setOpenConfirmDialog(false);
          }}
          onSubmit={handleCancelTaskClick}
        >
          <Typography>Are you sure you would like to cancel task?</Typography>
        </ConfirmationDialog>
      )}
    </>
  );
}
