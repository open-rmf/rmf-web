import {
  ApiServerModelsTortoiseModelsDeliveryAlertsDeliveryAlertLeaf as DeliveryAlert,
  TaskState,
} from 'api-client';
import React from 'react';
import { Button, TextField, Tooltip, Theme, Divider, useMediaQuery } from '@mui/material';
import Dialog from '@mui/material/Dialog';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import DialogTitle from '@mui/material/DialogTitle';
import { makeStyles, createStyles } from '@mui/styles';
import { base } from 'react-components';
import { AppControllerContext } from './app-contexts';
import { RmfAppContext } from './rmf-app';
import { TaskInspector } from './tasks/task-inspector';
import { TaskCancelButton } from './tasks/task-cancellation';

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    textField: {
      background: theme.palette.background.default,
      pointerEvents: 'none',
    },
  }),
);

const categoryToText = (category: string): string => {
  switch (category) {
    case 'missing': {
      return 'No cart detected';
    }
    case 'wrong': {
      return 'Wrong cart detected';
    }
    case 'obstructed': {
      return 'Goal is obstructed';
    }
    case 'cancelled': {
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
  onOverride?: (delivery_alert_id: string, task_id?: string) => Promise<void>;
  onResume?: (delivery_alert_id: string, task_id?: string) => Promise<void>;
  onClose: () => void;
}

const DeliveryWarningDialog = React.memo((props: DeliveryWarningDialogProps) => {
  const { deliveryAlert, taskState, onOverride, onResume, onClose } = props;
  const classes = useStyles();
  const [isOpen, setIsOpen] = React.useState(true);
  const [actionTaken, setActionTaken] = React.useState(!onOverride && !onResume);
  const [newTaskState, setNewTaskState] = React.useState<TaskState | null>(null);
  const [openTaskInspector, setOpenTaskInspector] = React.useState(false);
  const appController = React.useContext(AppControllerContext);
  const rmf = React.useContext(RmfAppContext);
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');

  React.useEffect(() => {
    if (deliveryAlert.action !== 'waiting') {
      setActionTaken(true);
    }
  }, [deliveryAlert]);

  React.useEffect(() => {
    if (!rmf) {
      console.error('Tasks api not available.');
      setNewTaskState(null);
      return;
    }
    if (!taskState) {
      setNewTaskState(null);
      return;
    }
    const sub = rmf.getTaskStateObs(taskState.booking.id).subscribe((taskStateUpdate) => {
      setNewTaskState(taskStateUpdate);
      if (
        deliveryAlert.action === 'waiting' &&
        taskStateUpdate.status &&
        taskStateUpdate.status === 'canceled'
      ) {
        (async () => {
          try {
            await rmf.deliveryAlertsApi.updateDeliveryAlertActionDeliveryAlertsDeliveryAlertIdActionPost(
              deliveryAlert.id,
              'cancelled',
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
  }, [rmf, deliveryAlert, taskState, appController]);

  const titleUpdateText = (action: string) => {
    switch (action) {
      case 'override': {
        return ' - [Overridden]';
      }
      case 'resume': {
        return ' - [Resumed]';
      }
      case 'cancel': {
        return ' - [Cancelled]';
      }
      case 'waiting':
      default: {
        return '';
      }
    }
  };

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
            }}
            InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 16 : 20 } }}
            InputProps={{ readOnly: true, className: classes.textField }}
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
            }}
            InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 16 : 20 } }}
            InputProps={{ readOnly: true, className: classes.textField }}
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
            }}
            InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 16 : 20 } }}
            InputProps={{ readOnly: true, className: classes.textField }}
            fullWidth={true}
            multiline
            maxRows={4}
            margin="dense"
            value={deliveryAlert.message ?? 'n/a'}
          />
        </DialogContent>
        <DialogActions>
          {(newTaskState && newTaskState.status && newTaskState.status === 'canceled') ||
          deliveryAlert.category === 'cancelled' ? (
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
                onOverride && onOverride(deliveryAlert.id, newTaskState?.booking.id);
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
                onResume && onResume(deliveryAlert.id, newTaskState?.booking.id);
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
  onClose: (delivery_alert_id: string) => Promise<void>;
}

const DeliveryErrorDialog = React.memo((props: DeliveryErrorDialogProps) => {
  const { deliveryAlert, taskState, onClose } = props;
  const classes = useStyles();
  const [isOpen, setIsOpen] = React.useState(true);
  const [openTaskInspector, setOpenTaskInspector] = React.useState(false);
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');

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
            }}
            InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 16 : 20 } }}
            InputProps={{ readOnly: true, className: classes.textField }}
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
            }}
            InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 16 : 20 } }}
            InputProps={{ readOnly: true, className: classes.textField }}
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
            }}
            InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 16 : 20 } }}
            InputProps={{ readOnly: true, className: classes.textField }}
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
              onClose(deliveryAlert.id);
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
  const rmf = React.useContext(RmfAppContext);
  const [alerts, setAlerts] = React.useState<Record<string, DeliveryAlertData>>({});
  const [closedErrorAlertId, setClosedErrorAlertId] = React.useState<string>();
  const appController = React.useContext(AppControllerContext);

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
      filteredAlerts[deliveryAlert.id] = {
        deliveryAlert,
        taskState,
      };
      return filteredAlerts;
    });
  };

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    // Initialize with any existing delivery alerts that are still waiting for
    // action.
    (async () => {
      let waitingDeliveryAlerts: DeliveryAlert[] = [];
      try {
        waitingDeliveryAlerts = (
          await rmf.deliveryAlertsApi.queryDeliveryAlertsDeliveryAlertsQueryGet(
            undefined,
            undefined,
            undefined,
            'waiting',
            undefined,
            undefined,
          )
        ).data;
      } catch (e) {
        console.error(`Failed to retrieve waiting delivery alerts: ${e}`);
        return;
      }

      const filteredAlertsMap: Record<string, DeliveryAlertData> = {};
      const taskIdToAlertsMap: Record<string, DeliveryAlertData> = {};
      for (const alert of waitingDeliveryAlerts) {
        // No task involved, and still waiting for user action. There should not
        // be any longstanding delivery alerts that appear after a refresh, only
        // the delivery alerts that are currently present and have not been
        // responded to.
        if (!alert.task_id) {
          filteredAlertsMap[alert.id] = { deliveryAlert: alert, taskState: undefined };
          continue;
        }

        // For the same task, ignore older alerts. This is possible as we are
        // creating delivery alert IDs with timestamps.
        const prevDeliveryAlert = taskIdToAlertsMap[alert.task_id];
        if (prevDeliveryAlert && prevDeliveryAlert.deliveryAlert.id > alert.id) {
          continue;
        }

        // Update map with newer alerts for the same task id, if it is still
        // unresolved.
        let state: TaskState | undefined = undefined;
        try {
          state = (await rmf.tasksApi.getTaskStateTasksTaskIdStateGet(alert.task_id)).data;
        } catch {
          console.error(
            `Failed to fetch task state for ${alert.task_id} for delivery alert ${alert.id}`,
          );
        }
        taskIdToAlertsMap[alert.task_id] = {
          deliveryAlert: alert,
          taskState: state,
        };
      }

      // Move all unresolved and up-to-date task related delivery alerts to the
      // filtered map.
      for (const alertData of Object.values(taskIdToAlertsMap)) {
        filteredAlertsMap[alertData.deliveryAlert.id] = alertData;
      }
      setAlerts(filteredAlertsMap);
    })();
  }, [rmf]);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = rmf.deliveryAlertObsStore.subscribe(async (deliveryAlert) => {
      // DEBUG
      console.log(
        `Got a delivery alert [${deliveryAlert.id}] with action [${deliveryAlert.action}]`,
      );

      let state: TaskState | undefined = undefined;
      if (deliveryAlert.task_id) {
        try {
          state = (await rmf.tasksApi.getTaskStateTasksTaskIdStateGet(deliveryAlert.task_id)).data;
        } catch {
          console.error(`Failed to fetch task state for ${deliveryAlert.task_id}`);
        }
      }
      // In the event that we are receiving the update of a closed error alert
      // that this dashboard instance introduced, we don't need to push it
      if (closedErrorAlertId && deliveryAlert.id === closedErrorAlertId) {
        return;
      }
      filterAndPushDeliveryAlert(deliveryAlert, state);
    });
    return () => sub.unsubscribe();
  }, [rmf, closedErrorAlertId]);

  const onOverride = React.useCallback<Required<DeliveryWarningDialogProps>['onOverride']>(
    async (delivery_alert_id, task_id) => {
      try {
        if (!rmf) {
          throw new Error('delivery alert api not available');
        }
        await rmf.deliveryAlertsApi.updateDeliveryAlertActionDeliveryAlertsDeliveryAlertIdActionPost(
          delivery_alert_id,
          'override',
        );
        const taskReferenceText = task_id ? `, continuing with task ${task_id}` : '';
        appController.showAlert(
          'success',
          `Overriding delivery alert ${delivery_alert_id}${taskReferenceText}`,
        );
      } catch (e) {
        const taskReferenceText = task_id ? ` and continue with task ${task_id}` : '';
        appController.showAlert(
          'error',
          `Failed to override delivery alert ${delivery_alert_id}${taskReferenceText}: ${
            (e as Error).message
          }`,
        );
      }
    },
    [rmf, appController],
  );

  const onResume = React.useCallback<Required<DeliveryWarningDialogProps>['onResume']>(
    async (delivery_alert_id, task_id) => {
      try {
        if (!rmf) {
          throw new Error('delivery alert api not available');
        }
        await rmf.deliveryAlertsApi.updateDeliveryAlertActionDeliveryAlertsDeliveryAlertIdActionPost(
          delivery_alert_id,
          'resume',
        );
        const taskReferenceText = task_id ? `, continuing with task ${task_id}` : '';
        appController.showAlert(
          'success',
          `Resuming after delivery alert ${delivery_alert_id}${taskReferenceText}`,
        );
      } catch (e) {
        const taskReferenceText = task_id ? ` ${task_id}` : '';
        appController.showAlert(
          'error',
          `Failed to resume task${taskReferenceText}: ${(e as Error).message}`,
        );
      }
    },
    [rmf, appController],
  );

  // Closing on an error requires the additional step of setting the action
  // to cancelled, as we want to ensure this error delivery dialog does not show
  // up anymore when the dashboard is refreshed.
  const onErrorCloseCancel = React.useCallback<Required<DeliveryErrorDialogProps>['onClose']>(
    async (delivery_alert_id) => {
      setClosedErrorAlertId(delivery_alert_id);

      // Check upstream alert action
      let alert: DeliveryAlert | undefined = undefined;
      try {
        if (!rmf) {
          throw new Error('delivery alert api not available');
        }
        alert = (
          await rmf.deliveryAlertsApi.getDeliveryAlertDeliveryAlertsDeliveryAlertIdGet(
            delivery_alert_id,
          )
        ).data;
      } catch (e) {
        console.error(`Failed to retrieve waiting delivery alerts: ${e}`);
        return;
      }

      // If alert is still waiting, we cancel it
      if (alert !== undefined && alert.action === 'waiting') {
        try {
          if (!rmf) {
            throw new Error('delivery alert api not available');
          }
          await rmf.deliveryAlertsApi.updateDeliveryAlertActionDeliveryAlertsDeliveryAlertIdActionPost(
            delivery_alert_id,
            'cancelled',
          );
        } catch (e) {
          console.error(
            `failed to update delivery alert ${delivery_alert_id} to cancelled action: ${
              (e as Error).message
            }`,
          );
        }
      }

      // Remove alert dialog from display
      setAlerts((prev) =>
        Object.fromEntries(Object.entries(prev).filter(([key]) => key !== delivery_alert_id)),
      );
    },
    [rmf],
  );

  return (
    <>
      {Object.values(alerts).map((alert) => {
        if (alert.deliveryAlert.tier === 'error') {
          return (
            <DeliveryErrorDialog
              deliveryAlert={alert.deliveryAlert}
              taskState={alert.taskState}
              onClose={onErrorCloseCancel}
              key={alert.deliveryAlert.id}
            />
          );
        }

        if (alert.deliveryAlert.category === 'cancelled') {
          console.warn(
            'Delivery alert with category [cancelled] submitted as a warning, this might be a mistake, alert promoted to an error.',
          );
          return (
            <DeliveryErrorDialog
              deliveryAlert={alert.deliveryAlert}
              taskState={alert.taskState}
              onClose={onErrorCloseCancel}
              key={alert.deliveryAlert.id}
            />
          );
        }

        // Allow resume if the obstruction is related to a latching problem.
        return (
          <DeliveryWarningDialog
            deliveryAlert={alert.deliveryAlert}
            taskState={alert.taskState}
            onOverride={alert.deliveryAlert.category === 'wrong' ? onOverride : undefined}
            onResume={
              alert.deliveryAlert.category !== 'obstructed'
                ? onResume
                : alert.deliveryAlert.message && alert.deliveryAlert.message.includes(' latch ')
                ? onResume
                : undefined
            }
            onClose={() =>
              setAlerts((prev) =>
                Object.fromEntries(
                  Object.entries(prev).filter(([key]) => key !== alert.deliveryAlert.id),
                ),
              )
            }
            key={alert.deliveryAlert.id}
          />
        );
      })}
    </>
  );
});
