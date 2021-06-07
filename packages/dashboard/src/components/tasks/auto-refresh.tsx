import { SioClient, Subscription } from 'api-client';
import React from 'react';
import * as RmfModels from 'rmf-models';

export interface AutoRefreshState {
  tasks: RmfModels.TaskSummary[];
  enabled: boolean;
}

export interface AutoRefreshDispatcher {
  setTasks: React.Dispatch<React.SetStateAction<RmfModels.TaskSummary[]>>;
  setEnabled: React.Dispatch<React.SetStateAction<boolean>>;
}

export function useAutoRefresh(
  sioClient?: SioClient,
  initalTasks: RmfModels.TaskSummary[] | (() => RmfModels.TaskSummary[]) = [],
  initialAutoRefresh: boolean | (() => boolean) = false,
): [AutoRefreshState, AutoRefreshDispatcher] {
  const [tasks, setTasks] = React.useState<RmfModels.TaskSummary[]>(initalTasks);
  const [taskIds, setTaskIds] = React.useState<string[]>(() => tasks.map((t) => t.task_id));
  const [enabled, setEnabled] = React.useState(initialAutoRefresh);

  React.useEffect(() => {
    if (!enabled || !sioClient) {
      return;
    }

    const subscriptions: Subscription[] = [];

    taskIds.forEach((taskId, idx) => {
      subscriptions.push(
        sioClient.subscribeTaskSummary(taskId, (newTask) =>
          setTasks((prev) => [...prev.slice(0, idx), newTask, ...prev.slice(idx + 1)]),
        ),
      );
    });

    return () => {
      subscriptions.forEach((s) => sioClient.unsubscribe(s));
    };
  }, [enabled, sioClient, taskIds]);

  const autoRefreshState = React.useMemo(() => ({ tasks, enabled }), [tasks, enabled]);
  const autoRefreshDispatcher = React.useMemo<AutoRefreshDispatcher>(
    () => ({
      setEnabled,
      setTasks: (value) => {
        if (typeof value === 'function') {
          setTasks((prev) => {
            const newTasks = value(prev);
            setTaskIds(newTasks.map((t) => t.task_id));
            return newTasks;
          });
        } else {
          const newTasks = value;
          setTaskIds(newTasks.map((t) => t.task_id));
          setTasks(value);
        }
      },
    }),
    [],
  );

  return [autoRefreshState, autoRefreshDispatcher];
}
