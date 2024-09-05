import { TableContainer } from '@mui/material';
import { TaskStateOutput as TaskState } from 'api-client';
import React from 'react';
import { RobotDataGridTable, RobotTableData } from 'react-components';

import { useRmfApi } from '../../hooks/use-rmf-api';
import { AppEvents } from '../app-events';
import { RobotSummary } from './robot-summary';

const RefreshRobotTableInterval = 10000;

export const RobotsTable = () => {
  const rmfApi = useRmfApi();

  const [robots, setRobots] = React.useState<Record<string, RobotTableData[]>>({});
  const [openRobotSummary, setOpenRobotSummary] = React.useState(false);
  const [selectedRobot, setSelectedRobot] = React.useState<RobotTableData>();

  React.useEffect(() => {
    const refreshRobotTable = async () => {
      const fleets = (await rmfApi.fleetsApi.getFleetsFleetsGet()).data;
      for (const fleet of fleets) {
        // fetch active tasks
        const taskIds = fleet.robots
          ? Object.values(fleet.robots).reduce<string[]>((acc, robot) => {
              if (robot.task_id) {
                acc.push(robot.task_id);
              }
              return acc;
            }, [])
          : [];

        const tasks =
          taskIds.length > 0
            ? (await rmfApi.tasksApi.queryTaskStatesTasksGet(taskIds.join(','))).data.reduce(
                (acc, task) => {
                  acc[task.booking.id] = task;
                  return acc;
                },
                {} as Record<string, TaskState>,
              )
            : {};

        setRobots((prev) => {
          if (!fleet.name) {
            return prev;
          }
          return {
            ...prev,
            [fleet.name]: fleet.robots
              ? Object.entries(fleet.robots).map<RobotTableData>(([name, robot]) => {
                  const estFinishTime = robot.task_id
                    ? tasks[robot.task_id].unix_millis_finish_time
                    : null;
                  return {
                    fleet: fleet.name || '',
                    name,
                    battery: robot.battery ? +robot.battery.toFixed(2) : undefined,
                    status: robot.status || undefined,
                    estFinishTime: estFinishTime || undefined,
                    lastUpdateTime: robot.unix_millis_time ? robot.unix_millis_time : undefined,
                    level: robot.location?.map || 'N/A',
                    commission: robot.commission || undefined,
                  };
                })
              : [],
          };
        });
      }
    };

    // Initialize table
    (async () => {
      await refreshRobotTable();
    })();

    // Set up the refresh trigger subscription
    const sub = AppEvents.refreshRobotApp.subscribe({
      next: async () => {
        await refreshRobotTable();
      },
    });

    // Set up regular interval to refresh table
    const refreshInterval = window.setInterval(refreshRobotTable, RefreshRobotTableInterval);
    return () => {
      clearInterval(refreshInterval);
      sub.unsubscribe();
    };
  }, [rmfApi]);

  return (
    <TableContainer sx={{ height: '100%' }}>
      <RobotDataGridTable
        robots={Object.values(robots).flatMap((r) => r)}
        onRobotClick={(_ev, robot) => {
          setOpenRobotSummary(true);
          AppEvents.robotSelect.next([robot.fleet, robot.name]);
          setSelectedRobot(robot);
        }}
      />
      {openRobotSummary && selectedRobot && (
        <RobotSummary robot={selectedRobot} onClose={() => setOpenRobotSummary(false)} />
      )}
    </TableContainer>
  );
};

export default RobotsTable;
