import {
  ApiServerModelsTortoiseModelsDeliveryAlertsDeliveryAlertLeaf as DeliveryAlert,
  TaskState,
} from 'api-client';
import React from 'react';
import { Button, TextField, Tooltip, Theme, Divider } from '@mui/material';
import Dialog from '@mui/material/Dialog';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import DialogTitle from '@mui/material/DialogTitle';
import { makeStyles, createStyles } from '@mui/styles';
import { base } from 'react-components';
import { AppControllerContext } from './app-contexts';
import { RmfAppContext } from './rmf-app';
import { TaskInspector } from './tasks/task-inspector';

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    textField: {
      background: theme.palette.background.default,
      pointerEvents: 'none',
    },
  }),
);

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
  const [cancelling, setCancelling] = React.useState(false);
  const appController = React.useContext(AppControllerContext);
  const rmf = React.useContext(RmfAppContext);

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
      setCancelling((prev) => {
        if (
          prev &&
          deliveryAlert.action === 'waiting' &&
          taskStateUpdate.status &&
          taskStateUpdate.status === 'canceled'
        ) {
          setCancelling(false);
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
        return prev;
      });

      if (
        deliveryAlert.action === 'waiting' &&
        taskStateUpdate.status &&
        taskStateUpdate.status === 'canceled'
      ) {
        setCancelling(false);
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

  const cancelTask = React.useCallback(
    async (task_id: string) => {
      if (!rmf) {
        console.error('Tasks api not available for task cancellation.');
        return;
      }
      await rmf.tasksApi.postCancelTaskTasksCancelTaskPost({
        type: 'cancel_task_request',
        task_id: task_id,
      });
    },
    [rmf],
  );

  return (
    <>
      <Dialog
        PaperProps={{
          style: {
            backgroundColor: base.palette.warning.dark,
            boxShadow: 'none',
          },
        }}
        maxWidth="sm"
        fullWidth={true}
        open={isOpen}
        key={deliveryAlert.id}
      >
        <DialogTitle align="center">Delivery - warning!</DialogTitle>
        <Divider />
        <DialogContent>
          <TextField
            label="Task ID"
            id="standard-size-small"
            size="small"
            variant="filled"
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
            InputProps={{ readOnly: true, className: classes.textField }}
            fullWidth={true}
            margin="dense"
            value={`${deliveryAlert.category === 'missing' ? 'No' : 'Wrong'} cart detected`}
          />
          <TextField
            label="Message"
            id="standard-size-small"
            size="small"
            variant="filled"
            InputProps={{ readOnly: true, className: classes.textField }}
            fullWidth={true}
            multiline
            maxRows={4}
            margin="dense"
            value={deliveryAlert.message ?? 'n/a'}
          />
        </DialogContent>
        <DialogActions>
          {newTaskState ? (
            <Tooltip title="Inspects the state and logs of the task.">
              <Button
                size="small"
                variant="contained"
                onClick={() => setOpenTaskInspector(true)}
                disabled={false}
                autoFocus
              >
                Inspect
              </Button>
            </Tooltip>
          ) : null}
          {newTaskState && newTaskState.status && newTaskState.status === 'canceled' ? (
            <Button size="small" variant="contained" disabled autoFocus>
              Cancelled
            </Button>
          ) : newTaskState ? (
            <Tooltip title="Cancels the current delivery task.">
              <Button
                size="small"
                variant="contained"
                disabled={actionTaken || cancelling}
                onClick={() => {
                  setCancelling(true);
                  if (newTaskState) {
                    const task_id = newTaskState.booking.id;
                    try {
                      cancelTask(task_id);
                      appController.showAlert('success', `Successfully cancelled task ${task_id}`);
                    } catch (e) {
                      appController.showAlert(
                        'error',
                        `Failed to cancel task ${task_id}: ${(e as Error).message}`,
                      );
                      setCancelling(false);
                    }
                  }
                }}
                autoFocus
              >
                {cancelling ? 'Cancelling...' : 'Cancel Delivery'}
              </Button>
            </Tooltip>
          ) : (
            <Button size="small" variant="contained" disabled autoFocus>
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

  return (
    <>
      <Dialog
        PaperProps={{
          style: {
            backgroundColor: base.palette.error.dark,
            boxShadow: 'none',
          },
        }}
        maxWidth="sm"
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
            InputProps={{ readOnly: true, className: classes.textField }}
            fullWidth={true}
            margin="dense"
            value={`${deliveryAlert.category === 'missing' ? 'No' : 'Wrong'} cart detected`}
          />
          <TextField
            label="Message"
            id="standard-size-small"
            size="small"
            variant="filled"
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
      // TODO(ac): Create an endpoint which only gives delivery alerts that have
      // not been actioned upon, to prevent requesting too many delivery alerts.
      let deliveryAlerts: DeliveryAlert[] = [];
      try {
        deliveryAlerts = (await rmf.deliveryAlertsApi.getDeliveryAlertsDeliveryAlertsGet()).data;
      } catch (e) {
        console.error(`Failed to retrieve existing delivery alerts: ${e}`);
        return;
      }

      const filteredAlertsMap: Record<string, DeliveryAlertData> = {};
      const taskIdToAlertsMap: Record<string, DeliveryAlertData> = {};
      for (const alert of deliveryAlerts) {
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

        // Update map with newer alerts for the same task id.
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
        if (alertData.deliveryAlert.action !== 'waiting') {
          continue;
        }
        filteredAlertsMap[alertData.deliveryAlert.id] = alertData;
      }
      setAlerts(filteredAlertsMap);
    })();

    const sub = rmf.deliveryAlertObsStore.subscribe(async (deliveryAlert) => {
      let state: TaskState | undefined = undefined;
      if (deliveryAlert.task_id) {
        try {
          state = (await rmf.tasksApi.getTaskStateTasksTaskIdStateGet(deliveryAlert.task_id)).data;
        } catch {
          console.error(`Failed to fetch task state for ${deliveryAlert.task_id}`);
        }
      }
      filterAndPushDeliveryAlert(deliveryAlert, state);
    });
    return () => sub.unsubscribe();
  }, [rmf]);

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
      setAlerts((prev) =>
        Object.fromEntries(Object.entries(prev).filter(([key]) => key !== delivery_alert_id)),
      );
    },
    [rmf],
  );

  return (
    <>
      {Object.values(alerts).map((alert) => {
        if (alert.deliveryAlert.tier === 'warning') {
          return (
            <DeliveryWarningDialog
              deliveryAlert={alert.deliveryAlert}
              taskState={alert.taskState}
              onOverride={alert.deliveryAlert.category === 'wrong' ? onOverride : undefined}
              onResume={onResume}
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
        }

        return (
          <DeliveryErrorDialog
            deliveryAlert={alert.deliveryAlert}
            taskState={alert.taskState}
            onClose={onErrorCloseCancel}
            key={alert.deliveryAlert.id}
          />
        );
      })}
    </>
  );
});
