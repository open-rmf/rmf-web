import { makeStyles } from '@material-ui/core';
import { TaskProgress } from 'api-client';
import React from 'react';
import { RobotPanel, TaskPanelProps } from 'react-components';
import * as RmfModels from 'rmf-models';
import { FleetStateContext, RmfIngressContext } from '../rmf-app';

const useStyles = makeStyles((theme) => ({
  robotPanel: {
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

export function RobotPage() {
  const classes = useStyles();
  const { tasksApi = null } = React.useContext(RmfIngressContext) || {};
  const fleetStates = React.useContext(FleetStateContext);
  const fleets = React.useMemo(() => Object.values(fleetStates), [fleetStates]);
  const fleetNames = React.useRef<string[]>([]);
  const newFleetNames = Object.keys(fleetStates);
  if (newFleetNames.some((fleetName) => !fleetNames.current.includes(fleetName))) {
    fleetNames.current = newFleetNames;
  }
  const [robotStates, setRobotStates] = React.useState<RmfModels.RobotState[]>([]);

  const fetchTasks = React.useCallback(
    async (limit: number, offset: number) => {
      if (!tasksApi) {
        return {
          tasks: [],
          totalCount: 0,
        };
      }
      const resp = await tasksApi.getTasksTasksGet(
        undefined,
        undefined,
        undefined,
        undefined,
        undefined,
        undefined,
        undefined,
        undefined,
        undefined,
        limit,
        offset,
        '-priority,-start_time',
      );
      const taskProgresses = resp.data.items;
      return taskProgresses;
    },
    [tasksApi],
  );

  React.useEffect(() => {
    const robotsStatesArray = fleets.flatMap((fleet, index) => {
      return fleet.robots;
    });
    setRobotStates(robotsStatesArray);
  }, [fleets]);

  return <RobotPanel className={classes.robotPanel} fetchTasks={fetchTasks} robots={robotStates} />;
}
