/* istanbul ignore file */

import { makeStyles } from '@material-ui/core';
import { TaskProgress } from 'api-client';
import React from 'react';
import { RobotPanel, RobotPanelProps } from 'react-components';
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

export function RobotPage() {
  const classes = useStyles();
  const { tasksApi = null, fleetsApi } = React.useContext(RmfIngressContext) || {};
  const fleetStates = React.useContext(FleetStateContext);
  const fleets = React.useMemo(() => Object.values(fleetStates), [fleetStates]);
  const fleetNames = React.useRef<string[]>([]);
  const newFleetNames = Object.keys(fleetStates);
  if (newFleetNames.some((fleetName) => !fleetNames.current.includes(fleetName))) {
    fleetNames.current = newFleetNames;
  }
  const [robotStates, setRobotStates] = React.useState<RmfModels.RobotState[]>([]);

  const test = async () => {
    const resp = await fleetsApi?.getRobotsFleetsRobotsGet(
      undefined,
      undefined,
      undefined,
      undefined,
      undefined,
      'start_time',
    );
    console.log(resp?.data.items);
  };

  test();

  const fetchTasks = React.useCallback<RobotPanelProps['fetchTasks']>(
    async (limit: number, offset: number) => {
      if (!tasksApi) {
        return [];
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
      const taskProgresses: TaskProgress[] = resp.data.items;
      return taskProgresses;
    },
    [tasksApi],
  );

  React.useEffect(() => {
    console.log('is this ever changing??????');
    const robotsStatesArray = fleets.flatMap((fleet, index) => {
      return fleet.robots;
    });
    setRobotStates(robotsStatesArray);
  }, [fleets]);

  return <RobotPanel className={classes.robotPanel} fetchTasks={fetchTasks} robots={robotStates} />;
}
