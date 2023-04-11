import React from 'react';
import { AlertStoreProps, AlertToDisplay } from '../alert-store';
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
import { Subscription } from 'rxjs';
import { AlertContent, AlertDialog } from 'react-components';
import { base } from 'react-components';

type Alert = ApiServerModelsTortoiseModelsAlertsAlertLeaf;

const showMessage = (task: TaskState | undefined) => {
  if (!task) {
    return 'No message';
  }

  switch (task.status) {
    case Status.Failed:
      return `${task.dispatch?.status}
                ${task.dispatch?.errors?.map((e) => e.message)}`;

    case Status.Completed:
      return 'Task completed!';

    default:
      return 'No message';
  }
};

const setTaskDialogColor = (taskStatus: Status | undefined) => {
  if (!taskStatus) {
    return base.palette.background.default;
  }

  switch (taskStatus) {
    case Status.Failed:
      return base.palette.error.dark;

    case Status.Completed:
      return base.palette.success.dark;

    default:
      return base.palette.background.default;
  }
};

const buildDialogContent = (alertToDisplay: AlertToDisplay): AlertContent[] => {
  return [
    {
      title: 'Task',
      value: alertToDisplay.task ? alertToDisplay.task.booking.id : '',
    },
    {
      title: 'Location',
      value: alertToDisplay.robot.location ? alertToDisplay.robot.location.map : '',
    },
    {
      title: 'Message',
      value: showMessage(alertToDisplay.task),
    },
  ];
};

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
            if (!ackResponse.acknowledged_by) {
              console.log(`Failed to acknowledge alert ID ${alert.task_id}`);
              showAlert('error', `Failed to acknowledge alert ID ${alert.task_id}`);
            } else {
              showAlert('success', `User acknowledged alert ID ${alert.task_id}`);
            }
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

export function TaskAlertComponent({ robots }: AlertStoreProps): JSX.Element {
  const rmf = React.useContext(RmfAppContext);
  const [alertsToDisplay, setAlertsToDisplay] = React.useState<AlertToDisplay[]>([]);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }

    const subs: Subscription[] = [];

    robots.map((r) => {
      if (r.robot?.task_id) {
        return subs.push(
          rmf.getTaskStateObs(r.robot.task_id).subscribe((task) => {
            if (task.status === Status.Completed || task.status === Status.Failed) {
              setAlertsToDisplay((prev) => [...prev, { show: true, task: task, robot: r.robot }]);
            }
          }),
        );
      }
      return [];
    });

    return () => subs.forEach((s) => s.unsubscribe());
  }, [rmf, robots]);

  return (
    <>
      {alertsToDisplay.map((r) =>
        r.show ? (
          <AlertDialog
            key={r.task ? r.task.booking.id : r.robot.name}
            // stopShowing={() =>
            //   setAlertsToDisplay((prev) =>
            //     prev.map((obj) => {
            //       if (obj.robot.name === r.robot.name) {
            //         return { ...obj, show: false };
            //       }
            //       return obj;
            //     }),
            //   )
            // }
            dismiss={() => {}}
            acknowledge={() => {}}
            title={'Task State'}
            progress={r.robot.battery ? r.robot.battery : -1}
            alertContents={buildDialogContent(r)}
            backgroundColor={setTaskDialogColor(r.task?.status)}
            // show={r.show}
          />
        ) : null,
      )}
    </>
  );
}
