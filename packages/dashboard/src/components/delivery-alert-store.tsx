import { Button, Divider, TextField, Tooltip, useMediaQuery, useTheme } from '@mui/material';
import Dialog from '@mui/material/Dialog';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import DialogTitle from '@mui/material/DialogTitle';
import {
  Action as DeliveryAlertAction,
  ApiServerModelsDeliveryAlertsDeliveryAlertCategory as DeliveryAlertCategory,
  ApiServerModelsDeliveryAlertsDeliveryAlertTier as DeliveryAlertTier,
  DeliveryAlert,
  TaskStateOutput as TaskState,
} from 'api-client';
import React from 'react';
import { base } from 'react-components';

import { useAppController } from '../hooks/use-app-controller';
import { useRmfApi } from '../hooks/use-rmf-api';
import { TaskCancelButton } from './tasks/task-cancellation';
import { TaskInspector } from './tasks/task-inspector';

const categoryToText = (category: DeliveryAlertCategory): string => {
  switch (category) {
    case DeliveryAlertCategory.Missing: {
      return 'No cart detected';
    }
    case DeliveryAlertCategory.Wrong: {
      return 'Wrong cart detected';
    }
    case DeliveryAlertCategory.Obstructed: {
      return 'Goal is obstructed';
    }
    case DeliveryAlertCategory.Cancelled: {
      return 'Task is cancelled';
    }
    default: {
      return '';
    }
  }
};

interface DeliveryWarningDialogProps {
  deliveryAlert: DeliveryAlert;
  taskState?: TaskState;
  onOverride?: (deliveryAlert: DeliveryAlert) => Promise<void>;
  onResume?: (deliveryAlert: DeliveryAlert) => Promise<void>;
  onClose: () => void;
}

const DeliveryWarningDialog = React.memo((props: DeliveryWarningDialogProps) => {
  const { deliveryAlert, taskState, onOverride, onResume, onClose } = props;
  const [isOpen, setIsOpen] = React.useState(true);
  const [actionTaken, setActionTaken] = React.useState(!onOverride && !onResume);
  const [newTaskState, setNewTaskState] = React.useState<TaskState | null>(null);
  const [openTaskInspector, setOpenTaskInspector] = React.useState(false);
  const appController = useAppController();
  const rmfApi = useRmfApi();
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');

  React.useEffect(() => {
    if (deliveryAlert.action !== DeliveryAlertAction.Waiting) {
      setActionTaken(true);
    }
  }, [deliveryAlert]);

  React.useEffect(() => {
    if (!taskState) {
      setNewTaskState(null);
      return;
    }
    const sub = rmfApi.getTaskStateObs(taskState.booking.id).subscribe((taskStateUpdate) => {
      setNewTaskState(taskStateUpdate);
      if (
        deliveryAlert.action === DeliveryAlertAction.Waiting &&
        taskStateUpdate.status &&
        taskStateUpdate.status === 'canceled'
      ) {
        (async () => {
          try {
            await rmfApi.deliveryAlertsApi.respondToDeliveryAlertDeliveryAlertsDeliveryAlertIdResponsePost(
              deliveryAlert.id,
              deliveryAlert.category,
              deliveryAlert.tier,
              deliveryAlert.task_id ?? '',
              DeliveryAlertAction.Cancel,
              deliveryAlert.message ?? '',
            );
          } catch (e) {
            appController.showAlert(
              'error',
              `Failed to cancel delivery alert ${deliveryAlert.id}: ${(e as Error).message}`,
            );
          }
          setActionTaken(true);
        })();
      }
    });
    return () => sub.unsubscribe();
  }, [rmfApi, deliveryAlert, taskState, appController]);

  const titleUpdateText = (action: DeliveryAlertAction) => {
    switch (action) {
      case DeliveryAlertAction.Override: {
        return ' - [Overridden]';
      }
      case DeliveryAlertAction.Resume: {
        return ' - [Resumed]';
      }
      case DeliveryAlertAction.Cancel: {
        return ' - [Cancelled]';
      }
      case DeliveryAlertAction.Waiting:
      default: {
        return '';
      }
    }
  };

  const theme = useTheme();

  return (
    <>
      <Dialog
        PaperProps={{
          style: {
            backgroundColor: base.palette.warning.dark,
            boxShadow: 'none',
          },
        }}
        maxWidth={isScreenHeightLessThan800 ? 'xs' : 'sm'}
        fullWidth={true}
        open={isOpen}
        key={deliveryAlert.id}
      >
        <DialogTitle align="center">
          Delivery - warning!{titleUpdateText(deliveryAlert.action)}
        </DialogTitle>
        <Divider />
        <DialogContent>
          <TextField
            label="Task ID"
            id="standard-size-small"
            size="small"
            variant="filled"
            sx={{
              '& .MuiFilledInput-root': {
                fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1.15',
              },
              background: theme.palette.background.default,
              pointerEvents: 'none',
            }}
            InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 16 : 20 } }}
            InputProps={{ readOnly: true }}
            fullWidth={true}
            margin="dense"
            value={
              deliveryAlert.task_id && deliveryAlert.task_id.length > 0
                ? deliveryAlert.task_id
                : 'n/a'
            }
          />
          <TextField
            label="Category"
            id="standard-size-small"
            size="small"
            variant="filled"
            sx={{
              '& .MuiFilledInput-root': {
                fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1.15',
              },
              background: theme.palette.background.default,
              pointerEvents: 'none',
            }}
            InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 16 : 20 } }}
            InputProps={{ readOnly: true }}
            fullWidth={true}
            margin="dense"
            value={categoryToText(deliveryAlert.category)}
          />
          <TextField
            label="Message"
            id="standard-size-small"
            size="small"
            variant="filled"
            sx={{
              '& .MuiFilledInput-root': {
                fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1.15',
              },
              background: theme.palette.background.default,
              pointerEvents: 'none',
            }}
            InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 16 : 20 } }}
            InputProps={{ readOnly: true }}
            fullWidth={true}
            multiline
            maxRows={4}
            margin="dense"
            value={deliveryAlert.message ?? 'n/a'}
          />
        </DialogContent>
        <DialogActions>
          {(newTaskState && newTaskState.status && newTaskState.status === 'canceled') ||
          deliveryAlert.category === DeliveryAlertCategory.Cancelled ? (
            <Button
              size="small"
              variant="contained"
              disabled
              autoFocus
              sx={{
                fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem',
                padding: isScreenHeightLessThan800 ? '4px 8px' : '6px 12px',
              }}
            >
              Cancelled
            </Button>
          ) : deliveryAlert.message && deliveryAlert.message.includes(' latch ') ? (
            <Button
              size="small"
              variant="contained"
              disabled
              autoFocus
              sx={{
                fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem',
                padding: isScreenHeightLessThan800 ? '4px 8px' : '6px 12px',
              }}
            >
              Cancel
            </Button>
          ) : newTaskState ? (
            <Tooltip title="Cancels the current delivery task.">
              <TaskCancelButton
                taskId={newTaskState.booking.id}
                size="small"
                variant="contained"
                color="secondary"
                disabled={actionTaken}
                buttonText={'Cancel Delivery'}
                sx={{
                  fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem',
                  padding: isScreenHeightLessThan800 ? '4px 8px' : '6px 12px',
                }}
              />
            </Tooltip>
          ) : (
            <Button
              size="small"
              variant="contained"
              disabled
              autoFocus
              sx={{
                fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem',
                padding: isScreenHeightLessThan800 ? '4px 8px' : '6px 12px',
              }}
            >
              Cancel Delivery
            </Button>
          )}
          <Tooltip title="Overrides the warning, the action will be performed regardless of it.">
            <Button
              size="small"
              variant="contained"
              disabled={onOverride === undefined || actionTaken}
              onClick={() => {
                setActionTaken(true);
                onOverride && onOverride(deliveryAlert);
              }}
              autoFocus
              sx={{
                fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem',
                padding: isScreenHeightLessThan800 ? '4px 8px' : '6px 12px',
              }}
            >
              Override!
            </Button>
          </Tooltip>
          <Tooltip title="Resumes the task, where the previous action will be attempted again.">
            <Button
              size="small"
              variant="contained"
              disabled={onResume === undefined || actionTaken}
              onClick={() => {
                setActionTaken(true);
                onResume && onResume(deliveryAlert);
              }}
              autoFocus
              sx={{
                fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem',
                padding: isScreenHeightLessThan800 ? '4px 8px' : '6px 12px',
              }}
            >
              Resume {'>>'}
            </Button>
          </Tooltip>
          <Button
            size="small"
            variant="contained"
            disabled={!actionTaken}
            onClick={() => {
              onClose();
              setIsOpen(false);
            }}
            autoFocus
            sx={{
              fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem',
              padding: isScreenHeightLessThan800 ? '4px 8px' : '6px 12px',
            }}
          >
            Close
          </Button>
        </DialogActions>
      </Dialog>
      {taskState && openTaskInspector && (
        <TaskInspector task={taskState} onClose={() => setOpenTaskInspector(false)} />
      )}
    </>
  );
});

interface DeliveryErrorDialogProps {
  deliveryAlert: DeliveryAlert;
  taskState?: TaskState;
  onClose: () => void;
}

const DeliveryErrorDialog = React.memo((props: DeliveryErrorDialogProps) => {
  const { deliveryAlert, taskState, onClose } = props;
  const [isOpen, setIsOpen] = React.useState(true);
  const [openTaskInspector, setOpenTaskInspector] = React.useState(false);
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');

  const theme = useTheme();

  return (
    <>
      <Dialog
        PaperProps={{
          style: {
            backgroundColor: base.palette.error.dark,
            boxShadow: 'none',
          },
        }}
        maxWidth={isScreenHeightLessThan800 ? 'xs' : 'sm'}
        fullWidth={true}
        open={isOpen}
        key={deliveryAlert.id}
      >
        <DialogTitle align="center">Delivery - error!</DialogTitle>
        <Divider />
        <DialogContent>
          <TextField
            label="Task ID"
            id="standard-size-small"
            size="small"
            variant="filled"
            sx={{
              '& .MuiFilledInput-root': {
                fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1.15',
              },
              background: theme.palette.background.default,
              pointerEvents: 'none',
            }}
            InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 16 : 20 } }}
            InputProps={{ readOnly: true }}
            fullWidth={true}
            margin="dense"
            value={deliveryAlert.task_id ?? 'n/a'}
          />
          <TextField
            label="Category"
            id="standard-size-small"
            size="small"
            variant="filled"
            sx={{
              '& .MuiFilledInput-root': {
                fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1.15',
              },
              background: theme.palette.background.default,
              pointerEvents: 'none',
            }}
            InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 16 : 20 } }}
            InputProps={{ readOnly: true }}
            fullWidth={true}
            margin="dense"
            value={categoryToText(deliveryAlert.category)}
          />
          <TextField
            label="Message"
            id="standard-size-small"
            size="small"
            variant="filled"
            sx={{
              '& .MuiFilledInput-root': {
                fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1.15',
              },
              background: theme.palette.background.default,
              pointerEvents: 'none',
            }}
            InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 16 : 20 } }}
            InputProps={{ readOnly: true }}
            fullWidth={true}
            multiline
            maxRows={4}
            margin="dense"
            value={deliveryAlert.message ?? 'n/a'}
          />
        </DialogContent>
        <DialogActions>
          {taskState ? (
            <Tooltip title="Inspects the state and logs of the task.">
              <Button
                size="small"
                variant="contained"
                onClick={() => setOpenTaskInspector(true)}
                disabled={false}
                autoFocus
                sx={{
                  fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem',
                  padding: isScreenHeightLessThan800 ? '4px 8px' : '6px 12px',
                }}
              >
                Inspect
              </Button>
            </Tooltip>
          ) : null}
          <Button
            size="small"
            variant="contained"
            onClick={() => {
              onClose();
              setIsOpen(false);
            }}
            disabled={false}
            autoFocus
            sx={{
              fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem',
              padding: isScreenHeightLessThan800 ? '4px 8px' : '6px 12px',
            }}
          >
            Close
          </Button>
        </DialogActions>
      </Dialog>
      {taskState && openTaskInspector && (
        <TaskInspector task={taskState} onClose={() => setOpenTaskInspector(false)} />
      )}
    </>
  );
});

interface DeliveryAlertData {
  deliveryAlert: DeliveryAlert;
  taskState?: TaskState;
}

export const DeliveryAlertStore = React.memo(() => {
  const rmfApi = useRmfApi();
  const [alerts, setAlerts] = React.useState<Record<string, DeliveryAlertData>>({});
  const appController = useAppController();

  const filterAndPushDeliveryAlert = (deliveryAlert: DeliveryAlert, taskState?: TaskState) => {
    // Check if a delivery alert for a task is already open, if so, replace it
    // with this new incoming deliveryAlert.
    setAlerts((prev) => {
      if (!deliveryAlert.task_id) {
        return {
          ...prev,
          [deliveryAlert.id]: { deliveryAlert, taskState },
        };
      }

      // TODO(ac): set action to cancelled for delivery alerts that have been
      // updated.
      const filteredAlerts = Object.fromEntries(
        Object.entries(prev).filter(
          ([_, alertData]) =>
            !alertData.deliveryAlert.task_id ||
            alertData.deliveryAlert.task_id !== deliveryAlert.task_id,
        ),
      );

      if (deliveryAlert.action === DeliveryAlertAction.Waiting) {
        filteredAlerts[deliveryAlert.id] = {
          deliveryAlert,
          taskState,
        };
      }
      return filteredAlerts;
    });
  };

  React.useEffect(() => {
    const sub = rmfApi.deliveryAlertObsStore.subscribe(async (deliveryAlert) => {
      // DEBUG
      console.log(
        `Got a delivery alert [${deliveryAlert.id}] with action [${deliveryAlert.action}]`,
      );

      let state: TaskState | undefined = undefined;
      if (deliveryAlert.task_id) {
        try {
          state = (await rmfApi.tasksApi.getTaskStateTasksTaskIdStateGet(deliveryAlert.task_id))
            .data;
        } catch {
          console.error(`Failed to fetch task state for ${deliveryAlert.task_id}`);
        }
      }
      filterAndPushDeliveryAlert(deliveryAlert, state);
    });
    return () => sub.unsubscribe();
  }, [rmfApi]);

  const onOverride = React.useCallback<Required<DeliveryWarningDialogProps>['onOverride']>(
    async (delivery_alert) => {
      try {
        await rmfApi.deliveryAlertsApi.respondToDeliveryAlertDeliveryAlertsDeliveryAlertIdResponsePost(
          delivery_alert.id,
          delivery_alert.category,
          delivery_alert.tier,
          delivery_alert.task_id ?? '',
          DeliveryAlertAction.Override,
          delivery_alert.message ?? '',
        );
        const taskReferenceText = delivery_alert.task_id
          ? `, continuing with task ${delivery_alert.task_id}`
          : '';
        appController.showAlert(
          'success',
          `Overriding delivery alert ${delivery_alert.id}${taskReferenceText}`,
        );
        removeDeliveryAlertDialog(delivery_alert.id);
      } catch (e) {
        const taskReferenceText = delivery_alert.task_id
          ? ` and continue with task ${delivery_alert.task_id}`
          : '';
        appController.showAlert(
          'error',
          `Failed to override delivery alert ${delivery_alert.id}${taskReferenceText}: ${
            (e as Error).message
          }`,
        );
      }
    },
    [rmfApi, appController],
  );

  const removeDeliveryAlertDialog = (id: string) => {
    setAlerts((prev) => Object.fromEntries(Object.entries(prev).filter(([key]) => key !== id)));
  };

  const onResume = React.useCallback<Required<DeliveryWarningDialogProps>['onResume']>(
    async (delivery_alert) => {
      try {
        await rmfApi.deliveryAlertsApi.respondToDeliveryAlertDeliveryAlertsDeliveryAlertIdResponsePost(
          delivery_alert.id,
          delivery_alert.category,
          delivery_alert.tier,
          delivery_alert.task_id ?? '',
          DeliveryAlertAction.Resume,
          delivery_alert.message ?? '',
        );
        const taskReferenceText = delivery_alert.task_id
          ? `, continuing with task ${delivery_alert.task_id}`
          : '';
        appController.showAlert(
          'success',
          `Resuming after delivery alert ${delivery_alert.id}${taskReferenceText}`,
        );
        removeDeliveryAlertDialog(delivery_alert.id);
      } catch (e) {
        const taskReferenceText = delivery_alert.task_id ? ` ${delivery_alert.task_id}` : '';
        appController.showAlert(
          'error',
          `Failed to resume task${taskReferenceText}: ${(e as Error).message}`,
        );
      }
    },
    [rmfApi, appController],
  );

  return (
    <>
      {Object.values(alerts).map((alert) => {
        if (alert.deliveryAlert.tier === DeliveryAlertTier.Error) {
          return (
            <DeliveryErrorDialog
              deliveryAlert={alert.deliveryAlert}
              taskState={alert.taskState}
              onClose={() => removeDeliveryAlertDialog(alert.deliveryAlert.id)}
              key={alert.deliveryAlert.id}
            />
          );
        }

        if (alert.deliveryAlert.category === DeliveryAlertCategory.Cancelled) {
          console.warn(
            'Delivery alert with category [cancelled] submitted as a warning, this might be a mistake, alert promoted to an error.',
          );
          return (
            <DeliveryErrorDialog
              deliveryAlert={alert.deliveryAlert}
              taskState={alert.taskState}
              onClose={() => removeDeliveryAlertDialog(alert.deliveryAlert.id)}
              key={alert.deliveryAlert.id}
            />
          );
        }

        // Allow resume if the obstruction is related to a latching problem.
        return (
          <DeliveryWarningDialog
            deliveryAlert={alert.deliveryAlert}
            taskState={alert.taskState}
            onOverride={
              alert.deliveryAlert.category === DeliveryAlertCategory.Wrong ? onOverride : undefined
            }
            onResume={
              alert.deliveryAlert.category !== DeliveryAlertCategory.Obstructed
                ? onResume
                : alert.deliveryAlert.message && alert.deliveryAlert.message.includes(' latch ')
                  ? onResume
                  : undefined
            }
            onClose={() => removeDeliveryAlertDialog(alert.deliveryAlert.id)}
            key={alert.deliveryAlert.id}
          />
        );
      })}
    </>
  );
});
