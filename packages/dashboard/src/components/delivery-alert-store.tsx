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
  onCancel?: () => void;
  onOverride?: () => void;
  onResume?: () => void;
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
                onCancel && onCancel();
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
                onOverride && onOverride();
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
                onResume && onResume();
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

        const onCancel = () => {};
        const onOverride = alert.deliveryAlert.category === 'wrong' ? () => {} : undefined;
        const onResume = () => {};

        return (
          <DeliveryWarningDialog
            deliveryAlert={alert.deliveryAlert}
            taskState={alert.taskState}
            onCancel={onCancel}
            onOverride={onOverride}
            onResume={onResume}
            onClose={onClose}
          />
        );
      })}
    </>
  );
});
