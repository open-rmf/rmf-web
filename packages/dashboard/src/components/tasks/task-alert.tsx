import React from 'react';
import {
  ApiServerModelsTortoiseModelsAlertsAlertLeaf,
  LogEntry,
  Status,
  TaskEventLog,
  TaskState,
  Tier,
} from 'api-client';
import { AppControllerContext } from '../app-contexts';
import { RmfAppContext } from '../rmf-app';
import { AlertContent, AlertDialog } from 'react-components';
import { base } from 'react-components';

type Alert = ApiServerModelsTortoiseModelsAlertsAlertLeaf;

interface TaskAlert extends TaskEventLog {
  title: string;
  progress?: number;
  content: AlertContent[];
  color: string;
}

export interface TaskAlertHandlerProps {
  alerts: Alert[];
  removeAlert: (id: string) => void;
}

export function TaskAlertHandler({ alerts, removeAlert }: TaskAlertHandlerProps): JSX.Element {
  const getErrorLogEntries = (logs: TaskEventLog) => {
    let errorLogs: LogEntry[] = [];
    if (logs.log) {
      errorLogs.concat(logs.log.filter((entry) => entry.tier === Tier.Error));
    }

    if (logs.phases) {
      for (let phase of Object.values(logs.phases)) {
        if (phase.log) {
          errorLogs.concat(phase.log.filter((entry) => entry.tier === Tier.Error));
        }
        if (phase.events) {
          for (let eventLogs of Object.values(phase.events)) {
            errorLogs.concat(eventLogs.filter((entry) => entry.tier === Tier.Error));
          }
        }
      }
    }
    return errorLogs;
  };

  const getAlertTitle = (state: TaskState, errorLogEntries: LogEntry[]) => {
    if (state.status && state.status === Status.Completed) {
      return 'Task completed';
    }
    if (errorLogEntries.length !== 0) {
      return 'Task error';
    }
    return 'Task alert';
  };

  const getTaskProgress = (state: TaskState) => {
    if (state.status && state.status === Status.Completed) {
      return 1;
    }

    if (!state.estimate_millis || !state.unix_millis_start_time || !state.unix_millis_finish_time) {
      return undefined;
    }
    return Math.min(
      1.0 - state.estimate_millis / (state.unix_millis_finish_time - state.unix_millis_start_time),
      1,
    );
  };

  const getAlertContent = (state: TaskState, errorLogEntries: LogEntry[]) => {
    // First field to be the taks ID
    let content: AlertContent[] = [
      {
        title: 'ID',
        value: state.booking.id,
      },
    ];

    // Second field would be any errors found
    if (errorLogEntries.length !== 0) {
      let consolidatedErrorMessages = '';
      for (let entry of errorLogEntries) {
        consolidatedErrorMessages += `${new Date(entry.unix_millis_time).toLocaleString()} - ${
          entry.text
        }\n`;
      }

      content = [
        ...content,
        {
          title: 'Error logs',
          value: consolidatedErrorMessages,
        },
      ];
    }

    // If the task happen to complete anyway, we mention that it has completed
    // in a separate log
    if (state.status && state.status === Status.Completed) {
      const completionTimeString = state.unix_millis_finish_time
        ? `${new Date(state.unix_millis_finish_time).toLocaleString()} - `
        : '';

      content = [
        ...content,
        {
          title: 'Logs',
          value: `${completionTimeString}Task completed!`,
        },
      ];
    }

    return content;
  };

  const getAlertColor = (state: TaskState, errorLogs: LogEntry[]) => {
    if (state.status) {
      switch (state.status) {
        case Status.Completed:
          return base.palette.success.dark;

        case Status.Error:
        case Status.Failed:
          return base.palette.error.dark;

        default:
          break;
      }
    }

    if (errorLogs.length !== 0) {
      return base.palette.error.dark;
    }

    return base.palette.background.default;
  };

  const rmf = React.useContext(RmfAppContext);
  const { showAlert } = React.useContext(AppControllerContext);
  const [taskAlerts, setTaskAlerts] = React.useState<Record<string, TaskAlert>>({});
  React.useEffect(() => {
    if (!rmf) {
      return;
    }

    for (let alert of alerts) {
      (async () => {
        try {
          const logs = (
            await rmf.tasksApi.getTaskLogTasksTaskIdLogGet(alert.id, `0,${Number.MAX_SAFE_INTEGER}`)
          ).data;
          const state = (await rmf.tasksApi.getTaskStateTasksTaskIdStateGet(alert.id)).data;

          if (logs && state) {
            const errorLogEntries = getErrorLogEntries(logs);
            setTaskAlerts((prev) => {
              return {
                ...prev,
                [alert.id]: {
                  title: getAlertTitle(state, errorLogEntries),
                  progress: getTaskProgress(state),
                  content: getAlertContent(state, errorLogEntries),
                  color: getAlertColor(state, errorLogEntries),
                  ...logs,
                },
              };
            });
          }
        } catch {
          console.log(`Failed to fetch task logs for ${alert.id}`);
        }
      })();
    }
  }, [rmf, alerts]);

  return (
    <>
      {Object.values(taskAlerts).map((alert) => {
        const dismissAlert = () => {
          removeAlert(alert.task_id);
        };
        const acknowledgeAlert = () => {
          if (!rmf) {
            throw new Error('alerts api not available');
          }
          (async () => {
            const ackResponse = (await rmf?.alertsApi.acknowledgeAlertAlertsIdPost(alert.task_id))
              .data;
            if (ackResponse.acknowledged_by && ackResponse.acknowledged_on) {
              // const ackMillisAgo = new Date() - new Date(ackResponse.acknowledged_on)
              showAlert(
                'success',
                // `User ${ackResponse.acknowledged_by} acknowledged alert ID ${alert.task_id} on ${ackResponse.acknowledged_on}`);
                `${ackResponse.acknowledged_by} - ${alert.task_id} - ${ackResponse.acknowledged_on}`,
              );
            } else {
              console.log(`Failed to acknowledge alert ID ${alert.task_id}`);
              showAlert('error', `Failed to acknowledge alert ID ${alert.task_id}`);
            }

            // if (!ackResponse.acknowledged_by) {
            //   console.log(`Failed to acknowledge alert ID ${alert.task_id}`);
            //   showAlert('error', `Failed to acknowledge alert ID ${alert.task_id}`);
            // } else {
            //   // const ackMillisAgo = new Date() - new Date(ackResponse.acknowledged_on)
            //   // const now = new Date();

            //   showAlert(
            //     'success',
            //     `User ${ackResponse.acknowledged_by} acknowledged alert ID ${alert.task_id} `);
            // }
          })();
        };

        return (
          <AlertDialog
            key={alert.task_id}
            dismiss={dismissAlert}
            acknowledge={acknowledgeAlert}
            title={alert.title}
            progress={alert.progress}
            alertContents={alert.content}
            backgroundColor={alert.color}
          />
        );
      })}
    </>
  );
}
