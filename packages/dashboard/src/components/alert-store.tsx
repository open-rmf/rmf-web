import {
  AlertRequest,
  ApiServerModelsAlertsAlertRequestTier,
  LogEntry,
  TaskEventLog,
  ApiServerModelsRmfApiLogEntryTier as LogEntryTier,
} from 'api-client';
import { AppEvents } from './app-events';
import {
  Button,
  Dialog,
  DialogActions,
  DialogTitle,
  TextField,
  Theme,
  Divider,
  useMediaQuery,
  DialogContent,
} from '@mui/material';
import { makeStyles, createStyles } from '@mui/styles';
import React from 'react';
import { base } from 'react-components';
import { AppControllerContext } from './app-contexts';
import { RmfAppContext } from './rmf-app';
import { Subscription } from 'rxjs';
import { TaskCancelButton } from './tasks/task-cancellation';

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    textField: {
      background: theme.palette.background.default,
      '&:hover': {
        backgroundColor: theme.palette.background.default,
      },
    },
  }),
);

interface AlertDialogProps {
  alertRequest: AlertRequest;
  onDismiss: () => void;
}

const AlertDialog = React.memo((props: AlertDialogProps) => {
  const { alertRequest, onDismiss } = props;
  const classes = useStyles();
  const [isOpen, setIsOpen] = React.useState(true);
  const { showAlert } = React.useContext(AppControllerContext);
  const rmf = React.useContext(RmfAppContext);
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');
  const [additionalAlertMessage, setAdditionalAlertMessage] = React.useState<string | null>(null);

  const respondToAlert = async (alert_id: string, response: string) => {
    if (!rmf) {
      return;
    }

    try {
      const resp = (
        await rmf.alertsApi.respondToAlertAlertsRequestAlertIdRespondPost(alert_id, response)
      ).data;
      console.log(
        `Alert [${alertRequest.id}]: responded with [${resp.response}] at ${resp.unix_millis_response_time}`,
      );
    } catch (e) {
      const errorMessage = `Failed to respond [${response}] to alert ID [${alertRequest.id}], ${
        (e as Error).message
      }`;
      console.error(errorMessage);
      showAlert('error', errorMessage);
      return;
    }

    const successMessage = `Responded [${response}] to alert ID [${alertRequest.id}]`;
    console.log(successMessage);
    showAlert('success', successMessage);
  };

  const getErrorLogEntries = (logs: TaskEventLog) => {
    let errorLogs: LogEntry[] = [];
    if (logs.log) {
      errorLogs.concat(logs.log.filter((entry) => entry.tier === LogEntryTier.Error));
    }

    if (logs.phases) {
      for (let phase of Object.values(logs.phases)) {
        if (phase.log) {
          errorLogs.concat(phase.log.filter((entry) => entry.tier === LogEntryTier.Error));
        }
        if (phase.events) {
          for (let eventLogs of Object.values(phase.events)) {
            errorLogs.concat(eventLogs.filter((entry) => entry.tier === LogEntryTier.Error));
          }
        }
      }
    }
    return errorLogs;
  };

  React.useEffect(() => {
    if (alertRequest.tier === ApiServerModelsAlertsAlertRequestTier.Info || !alertRequest.task_id) {
      return;
    }
    if (!rmf) {
      return;
    }

    (async () => {
      if (!alertRequest.task_id) {
        return;
      }

      let logs: TaskEventLog | null = null;
      try {
        logs = (
          await rmf.tasksApi.getTaskLogTasksTaskIdLogGet(
            alertRequest.task_id,
            `0,${Number.MAX_SAFE_INTEGER}`,
          )
        ).data;
      } catch {
        console.log(
          `Failed to fetch task [${alertRequest.task_id}] logs for alert [${alertRequest.id}]`,
        );
      }
      const errorLogEntries = logs ? getErrorLogEntries(logs) : [];

      let consolidatedErrorMessages = '';
      for (const entry of errorLogEntries) {
        consolidatedErrorMessages += `${new Date(entry.unix_millis_time).toLocaleString()} - ${
          entry.text
        }\n`;
      }
      if (consolidatedErrorMessages.length > 0) {
        setAdditionalAlertMessage(consolidatedErrorMessages);
      }
    })();
  }, [rmf, alertRequest.id, alertRequest.task_id, alertRequest.tier]);

  return (
    <>
      <Dialog
        PaperProps={{
          style: {
            backgroundColor:
              alertRequest.tier === ApiServerModelsAlertsAlertRequestTier.Info
                ? base.palette.success.dark
                : alertRequest.tier === ApiServerModelsAlertsAlertRequestTier.Warning
                ? base.palette.warning.dark
                : base.palette.error.dark,
            boxShadow: 'none',
          },
        }}
        maxWidth={isScreenHeightLessThan800 ? 'xs' : 'sm'}
        fullWidth={true}
        open={isOpen}
        key={alertRequest.id}
      >
        <DialogTitle align="center">
          {alertRequest.title.length > 0 ? alertRequest.title : 'n/a'}
        </DialogTitle>
        <Divider />
        <DialogContent>
          <TextField
            label="Subtitle"
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
            value={alertRequest.subtitle.length > 0 ? alertRequest.subtitle : 'n/a'}
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
            value={
              (alertRequest.message.length > 0 ? alertRequest.message : 'n/a') +
              '\n' +
              (additionalAlertMessage ?? '')
            }
          />
        </DialogContent>
        <DialogActions>
          {alertRequest.responses_available.map((response) => {
            return (
              <Button
                size="small"
                variant="contained"
                autoFocus
                key={`${alertRequest.id}-${response}`}
                sx={{
                  fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem',
                  padding: isScreenHeightLessThan800 ? '4px 8px' : '6px 12px',
                }}
                onClick={async () => {
                  await respondToAlert(alertRequest.id, response);
                  AppEvents.refreshAlert.next();
                  setIsOpen(false);
                }}
              >
                {response}
              </Button>
            );
          })}
          {alertRequest.task_id ? (
            <TaskCancelButton
              taskId={alertRequest.task_id}
              size="small"
              variant="contained"
              color="secondary"
              buttonText={'Cancel task'}
              sx={{
                fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem',
                padding: isScreenHeightLessThan800 ? '4px 8px' : '6px 12px',
              }}
            />
          ) : null}
          <Button
            size="small"
            variant="contained"
            autoFocus
            sx={{
              fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem',
              padding: isScreenHeightLessThan800 ? '4px 8px' : '6px 12px',
            }}
            onClick={() => {
              onDismiss();
              setIsOpen(false);
            }}
          >
            Dismiss
          </Button>
        </DialogActions>
      </Dialog>
    </>
  );
});

export const AlertStore = React.memo(() => {
  const rmf = React.useContext(RmfAppContext);
  const [openAlerts, setOpenAlerts] = React.useState<Record<string, AlertRequest>>({});

  React.useEffect(() => {
    if (!rmf) {
      return;
    }

    const pushAlertIfUnresponded = async (alertRequest: AlertRequest) => {
      if (!rmf) {
        return;
      }
      try {
        const resp = (
          await rmf.alertsApi.getAlertResponseAlertsRequestAlertIdResponseGet(alertRequest.id)
        ).data;
        console.log(
          `Alert [${alertRequest.id}]: was responded with [${resp.response}] at ${resp.unix_millis_response_time}`,
        );
      } catch (e) {
        console.log(
          `Alert response could not be found for ${alertRequest.id}, ${(e as Error).message}`,
        );
        setOpenAlerts((prev) => {
          return {
            ...prev,
            [alertRequest.id]: alertRequest,
          };
        });
        AppEvents.refreshAlert.next();
        return;
      }
    };

    const subs: Subscription[] = [];

    subs.push(
      rmf.alertRequestsObsStore.subscribe(
        async (alertRequest) => await pushAlertIfUnresponded(alertRequest),
      ),
    );

    subs.push(
      AppEvents.alertListOpenedAlert.subscribe(async (alertRequest) => {
        if (!alertRequest) {
          return;
        }
        await pushAlertIfUnresponded(alertRequest);
      }),
    );

    subs.push(
      rmf.alertResponsesObsStore.subscribe((alertResponse) => {
        setOpenAlerts((prev) => {
          return Object.fromEntries(
            Object.entries(prev).filter(([key]) => key !== alertResponse.id),
          );
        });
        AppEvents.refreshAlert.next();
      }),
    );

    return () => {
      for (const sub of subs) {
        sub.unsubscribe();
      }
    };
  }, [rmf]);

  const removeOpenAlert = (id: string) => {
    const filteredAlerts = Object.fromEntries(
      Object.entries(openAlerts).filter(([key]) => key !== id),
    );
    setOpenAlerts(filteredAlerts);
  };

  return (
    <>
      {Object.values(openAlerts).map((alert) => {
        const removeThisAlert = () => {
          removeOpenAlert(alert.id);
        };
        return <AlertDialog key={alert.id} alertRequest={alert} onDismiss={removeThisAlert} />;
      })}
    </>
  );
});
