import React from 'react';
import {
  ApiServerModelsRmfApiLogEntryTier as Tier,
  Alert,
  LogEntry,
  ApiServerModelsRmfApiTaskStateStatus as Status,
  Status2 as DispatchTaskStatus,
  TaskEventLog,
  TaskStateOutput as TaskState,
} from 'api-client';
import { AppControllerContext } from '../app-contexts';
import { RmfAppContext } from '../rmf-app';
import { AlertContent, AlertDialog } from 'react-components';
import { base } from 'react-components';
import { TaskInspector } from './task-inspector';

interface TaskAlert extends TaskEventLog {
  title: string;
  progress?: number;
  content: AlertContent[];
  color: string;
  acknowledgedBy?: string;
  late: boolean;
}

export interface TaskAlertDialogProps {
  alert: Alert;
  removeAlert: () => void;
}

export function TaskAlertDialog({ alert, removeAlert }: TaskAlertDialogProps): JSX.Element {
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
    if (state.status && state.status === Status.Failed) {
      return 'Task failed';
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

    // Second field would be any dispatch errors
    if (state.dispatch && state.dispatch.status === DispatchTaskStatus.FailedToAssign) {
      let errors = '';
      if (state.dispatch.errors) {
        for (const error of state.dispatch.errors) {
          errors += `${JSON.stringify(error)}\n`;
        }
      }
      content = [
        ...content,
        {
          title: 'Dispatch failure',
          value: errors.length !== 0 ? errors : 'n/a',
        },
      ];
    }

    // Third field would be any errors found
    if (errorLogEntries.length !== 0) {
      let consolidatedErrorMessages = '';
      for (let entry of errorLogEntries) {
        consolidatedErrorMessages += `${new Date(entry.unix_millis_time).toLocaleString()} - ${
          entry.text
        }
          \n`;
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
  const [taskAlert, setTaskAlert] = React.useState<TaskAlert | null>(null);
  const [openTaskInspector, setOpenTaskInspector] = React.useState(false);
  const [taskState, setTaskState] = React.useState<TaskState | null>(null);
  React.useEffect(() => {
    if (!rmf) {
      return;
    }

    (async () => {
      // TODO(AC): Move away from using substrings, perhaps delayed tasks will
      // be its own category.
      const late_substr = '__late';
      const task_late = alert.original_id.includes(late_substr);
      const task_id = task_late
        ? alert.original_id.substring(0, alert.original_id.length - late_substr.length)
        : alert.original_id;

      let logs: TaskEventLog | null = null;
      try {
        logs = (
          await rmf.tasksApi.getTaskLogTasksTaskIdLogGet(task_id, `0,${Number.MAX_SAFE_INTEGER}`)
        ).data;
      } catch {
        console.log(`Failed to fetch task logs for ${alert.original_id}`);
      }
      const errorLogEntries = logs ? getErrorLogEntries(logs) : [];

      let state: TaskState | null = null;
      try {
        state = (await rmf.tasksApi.getTaskStateTasksTaskIdStateGet(task_id)).data;
      } catch {
        console.log(`Failed to fetch task state for ${alert.original_id}`);
      }
      setTaskState(state);

      let acknowledgedBy: string | undefined = undefined;
      if (alert.acknowledged_by) {
        acknowledgedBy = alert.acknowledged_by;
      } else if (alert.unix_millis_acknowledged_time) {
        acknowledgedBy = '-';
      }

      if (task_late && state) {
        setTaskAlert({
          title: getAlertTitle(state, []),
          progress: getTaskProgress(state),
          content: getAlertContent(state, []),
          color: getAlertColor(state, []),
          acknowledgedBy: acknowledgedBy,
          late: task_late,
          task_id: task_id,
        });
      } else if (!task_late && logs && state) {
        setTaskAlert({
          title: getAlertTitle(state, errorLogEntries),
          progress: getTaskProgress(state),
          content: getAlertContent(state, errorLogEntries),
          color: getAlertColor(state, errorLogEntries),
          acknowledgedBy: acknowledgedBy,
          late: task_late,
          ...logs,
        });
      } else if (state) {
        setTaskAlert({
          title: getAlertTitle(state, []),
          progress: getTaskProgress(state),
          content: getAlertContent(state, []),
          color: getAlertColor(state, []),
          acknowledgedBy: acknowledgedBy,
          late: task_late,
          task_id: state.booking.id,
        });
      }
    })();
  }, [rmf, alert.original_id, alert.acknowledged_by, alert.unix_millis_acknowledged_time]);

  const acknowledgeAlert = () => {
    if (!rmf) {
      throw new Error('alerts api not available');
    }
    if (!taskAlert) {
      return;
    }

    (async () => {
      const idToAcknowledge = taskAlert.late ? `${taskAlert.task_id}__late` : taskAlert.task_id;
      try {
        const ackResponse = (
          await rmf?.alertsApi.acknowledgeAlertAlertsAlertIdPost(idToAcknowledge)
        ).data;
        if (ackResponse.id !== ackResponse.original_id) {
          let showAlertMessage = `Alert ${ackResponse.original_id} acknowledged`;
          if (ackResponse.acknowledged_by) {
            showAlertMessage += ` by User ${ackResponse.acknowledged_by}`;
          }
          if (ackResponse.unix_millis_acknowledged_time) {
            const ackSecondsAgo =
              (new Date().getTime() - ackResponse.unix_millis_acknowledged_time) / 1000;
            showAlertMessage += ` ${Math.round(ackSecondsAgo)}s ago`;
          }
          showAlert('success', showAlertMessage);
          removeAlert();
        } else {
          throw new Error(`Failed to acknowledge alert ID ${idToAcknowledge}`);
        }
      } catch (error) {
        showAlert('error', `Failed to acknowledge alert ID ${idToAcknowledge}`);
        console.log(error);
      }
    })();
  };

  if (!taskAlert) {
    return <></>;
  }

  return (
    <>
      <AlertDialog
        key={taskAlert.task_id}
        onDismiss={removeAlert}
        acknowledgedBy={taskAlert.acknowledgedBy}
        onAcknowledge={taskAlert.acknowledgedBy ? undefined : acknowledgeAlert}
        title={taskAlert.title}
        progress={taskAlert.progress}
        alertContents={taskAlert.content}
        backgroundColor={taskAlert.color}
      />
      {openTaskInspector && (
        <TaskInspector task={taskState} onClose={() => setOpenTaskInspector(false)} />
      )}
    </>
  );
}
