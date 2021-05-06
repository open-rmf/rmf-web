import { makeStyles } from '@material-ui/core';
import { Task } from 'api-client';
import React from 'react';
import { TaskPanel, TaskPanelProps } from 'react-components';
import * as RmfModels from 'rmf-models';
import { PlacesContext, RmfIngressContext } from '../rmf-app';

const useStyles = makeStyles((theme) => ({
  taskPanel: {
    margin: `${theme.spacing(4)}px auto`,
    width: '100%',
    height: '100%',
    maxWidth: 1600,
  },
}));

/**
 * Sort tasks in place, by priority, then by start time.
 */
function sortTasks(tasks: RmfModels.TaskSummary[]) {
  tasks.sort((a, b) => {
    const aPriority = a.task_profile.description.priority.value;
    const bPriority = b.task_profile.description.priority.value;
    if (aPriority === bPriority) {
      const aStartTime = a.start_time.sec;
      const bStartTime = b.start_time.sec;
      return aStartTime - bStartTime;
    }
    return aPriority - bPriority;
  });
  return tasks;
}

export function TaskPage() {
  const classes = useStyles();
  const { tasksApi = null } = React.useContext(RmfIngressContext) || {};
  const places = React.useContext(PlacesContext);
  const [taskSummaries, setTaskSummaries] = React.useState<RmfModels.TaskSummary[]>([]);

  const handleRefresh = React.useCallback(async () => {
    if (!tasksApi) {
      return;
    }
    const tasks = await tasksApi.getTasksTasksGetTasksGet();
    const getTaskSummaries = tasks.data.map((task: Task) => task.task_summary);
    sortTasks(getTaskSummaries);
    setTaskSummaries(getTaskSummaries);
  }, [tasksApi]);

  React.useEffect(() => {
    handleRefresh();
  }, [handleRefresh]);

  const submitTask = React.useCallback<Required<TaskPanelProps>['submitTask']>(
    async (body) => {
      await tasksApi?.submitTaskTasksSubmitTaskPost(body);
      handleRefresh();
    },
    [tasksApi, handleRefresh],
  );
  return (
    <TaskPanel
      className={classes.taskPanel}
      tasks={taskSummaries}
      cleaningZones={Object.keys(places)}
      loopWaypoints={Object.keys(places)}
      deliveryWaypoints={Object.keys(places)}
      submitTask={submitTask}
      onRefreshClick={handleRefresh}
    />
  );
}
