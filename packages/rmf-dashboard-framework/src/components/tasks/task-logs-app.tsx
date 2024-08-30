import { CardContent } from '@mui/material';
import { TaskEventLog, TaskStateOutput as TaskState } from 'api-client';
import React from 'react';

import { useRmfApi } from '../../hooks/use-rmf-api';
import { AppEvents } from '../app-events';
import { TaskLogs } from './task-logs';

export const TaskLogsCard = () => {
  const rmfApi = useRmfApi();
  const [taskState, setTaskState] = React.useState<TaskState | null>(null);
  const [taskLogs, setTaskLogs] = React.useState<TaskEventLog | null>(null);
  React.useEffect(() => {
    const sub = AppEvents.taskSelect.subscribe((task) => {
      if (!task) {
        setTaskState(null);
        setTaskLogs(null);
        return;
      }
      (async () => {
        // TODO: Get full logs, then subscribe to log updates for new logs.
        // Unlike with state events, we can't just subscribe to logs updates.
        try {
          const logs = (
            await rmfApi.tasksApi.getTaskLogTasksTaskIdLogGet(
              task.booking.id,
              `0,${Number.MAX_SAFE_INTEGER}`,
            )
          ).data;
          setTaskLogs(logs);
        } catch {
          console.log(`Failed to fetch task logs for ${task.booking.id}`);
          setTaskLogs(null);
        }
        setTaskState(task);
      })();
    });
    return () => sub.unsubscribe();
  }, [rmfApi]);

  return (
    <CardContent>
      <TaskLogs taskLog={taskLogs} taskState={taskState} />
    </CardContent>
  );
};

export default TaskLogsCard;
