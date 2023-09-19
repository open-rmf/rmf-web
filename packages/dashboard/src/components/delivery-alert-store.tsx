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
  onCancel?: (delivery_alert_id: string, task_id: string) => Promise<void>;
  onOverride?: (delivery_alert_id: string, task_id?: string) => Promise<void>;
  onResume?: (delivery_alert_id: string, task_id?: string) => Promise<void>;
  onClose: () => void;
}

const DeliveryWarningDialog = React.memo((props: DeliveryWarningDialogProps) => {
  const { deliveryAlert, taskState, onCancel, onOverride, onResume, onClose } = props;
  const classes = useStyles();
  const [isOpen, setIsOpen] = React.useState(true);
  const [actionTaken, setActionTaken] = React.useState(!onCancel && !onOverride && !onResume);
  const [openTaskInspector, setOpenTaskInspector] = React.useState(false);

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
          <Tooltip title="Cancels the current delivery task.">
            <Button
              size="small"
              variant="contained"
              disabled={onCancel === undefined || actionTaken}
              onClick={() => {
                setActionTaken(true);
                taskState && onCancel && onCancel(deliveryAlert.id, taskState.booking.id);
              }}
              autoFocus
            >
              Cancel Delivery
            </Button>
          </Tooltip>
          <Tooltip title="Overrides the warning, the action will be performed regardless of it.">
            <Button
              size="small"
              variant="contained"
              disabled={onOverride === undefined || actionTaken}
              onClick={() => {
                setActionTaken(true);
                onOverride && onOverride(deliveryAlert.id, taskState?.booking.id);
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
                onResume && onResume(deliveryAlert.id, taskState?.booking.id);
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
  onClose: () => void;
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
              onClose();
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

  // What happens if the user just refreshes?
  // Run the gets command the first time and publish all the alerts
  // TODO(ac): Create an endpoint which only gives delivery alerts that have not
  // been actioned upon, to prevent requesting too many delivery alerts.

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

      const filteredAlerts = Object.fromEntries(
        Object.entries(prev).filter(
          ([id, alertData]) =>
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
    const sub = rmf.deliveryAlertObsStore.subscribe(async (deliveryAlert) => {
      let state: TaskState | undefined = undefined;
      if (deliveryAlert.task_id) {
        try {
          state = (await rmf.tasksApi.getTaskStateTasksTaskIdStateGet(deliveryAlert.task_id)).data;
        } catch {
          console.log(`Failed to fetch task state for ${deliveryAlert.task_id}`);
        }
      }
      filterAndPushDeliveryAlert(deliveryAlert, state);
    });
    return () => sub.unsubscribe();
  }, [rmf]);

  const removeDeliveryAlertData = (id: string) => {
    const filteredAlerts = Object.fromEntries(Object.entries(alerts).filter(([key]) => key !== id));
    setAlerts(filteredAlerts);
  };

  const onCancel = React.useCallback<Required<DeliveryWarningDialogProps>['onCancel']>(
    async (delivery_alert_id, task_id) => {
      try {
        if (!rmf) {
          throw new Error('tasks and delivery alert api not available');
        }
        await rmf.tasksApi?.postCancelTaskTasksCancelTaskPost({
          type: 'cancel_task_request',
          task_id: task_id,
        });
        await rmf.deliveryAlertsApi?.updateDeliveryAlertActionDeliveryAlertsDeliveryAlertIdActionPost(
          delivery_alert_id,
          'cancelled',
        );
        appController.showAlert('success', 'Successfully cancelled task');
      } catch (e) {
        appController.showAlert('error', `Failed to cancel task: ${(e as Error).message}`);
      }
    },
    [rmf, appController],
  );

  const onOverride = React.useCallback<Required<DeliveryWarningDialogProps>['onOverride']>(
    async (delivery_alert_id, task_id) => {
      try {
        if (!rmf) {
          throw new Error('delivery alert api not available');
        }
        await rmf.deliveryAlertsApi?.updateDeliveryAlertActionDeliveryAlertsDeliveryAlertIdActionPost(
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
        await rmf.deliveryAlertsApi?.updateDeliveryAlertActionDeliveryAlertsDeliveryAlertIdActionPost(
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

  return (
    <>
      {Object.values(alerts).map((alert) => {
        const onClose = () => {
          removeDeliveryAlertData(alert.deliveryAlert.id);
        };

        if (alert.deliveryAlert.tier === 'error') {
          return (
            <DeliveryErrorDialog
              deliveryAlert={alert.deliveryAlert}
              taskState={alert.taskState}
              onClose={onClose}
            />
          );
        }

        return (
          <DeliveryWarningDialog
            deliveryAlert={alert.deliveryAlert}
            taskState={alert.taskState}
            onCancel={alert.taskState ? onCancel : undefined}
            onOverride={onOverride}
            onResume={onResume}
            onClose={onClose}
          />
        );
      })}
    </>
  );
});
