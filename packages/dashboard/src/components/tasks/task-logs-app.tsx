import { Box } from '@mui/material';
import { TaskEventLog, TaskState } from 'api-client';
import React from 'react';
import { AppEvents } from '../app-events';
import { createMicroApp } from '../micro-app';
import { RmfAppContext } from '../rmf-app';
import { TaskLogs } from './task-logs';

export const TaskLogsApp = createMicroApp('Task Logs', () => {
  const rmf = React.useContext(RmfAppContext);
  const [taskState, setTaskState] = React.useState<TaskState | null>(null);
  const [taskLogs, setTaskLogs] = React.useState<TaskEventLog | null>(null);
  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = AppEvents.taskSelect.subscribe((task) => {
      if (!task) {
        setTaskLogs(null);
        setTaskLogs(null);
        return;
      }
      (async () => {
        // TODO: Get full logs, then subscribe to log updates for new logs.
        // Unlike with state events, we can't just subscribe to logs updates.
        const logs = (
          await rmf.tasksApi.getTaskLogTasksTaskIdLogGet(
            task.booking.id,
            `0,${Number.MAX_SAFE_INTEGER}`,
          )
        ).data;
        setTaskState(task);
        setTaskLogs(logs);
      })();
    });
    return () => sub.unsubscribe();
  }, [rmf]);

  return (
    <Box>
      {taskLogs && taskState ? <TaskLogs taskLog={taskLogs} taskState={taskState} /> : null}
    </Box>
  );
});
