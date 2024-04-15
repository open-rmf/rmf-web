import { TableContainer } from '@mui/material';
import { TaskState } from 'api-client';
import React from 'react';
import { RobotDataGridTable, RobotTableData } from 'react-components';
import { AppEvents } from '../app-events';
import { createMicroApp } from '../micro-app';
import { RmfAppContext } from '../rmf-app';
import { RobotSummary } from './robot-summary';

const RefreshRobotTableInterval = 10000;

export const RobotsApp = createMicroApp('Robots', () => {
  const rmf = React.useContext(RmfAppContext);

  const [robots, setRobots] = React.useState<Record<string, RobotTableData[]>>({});
  const [openRobotSummary, setOpenRobotSummary] = React.useState(false);
  const [selectedRobot, setSelectedRobot] = React.useState<RobotTableData>();

  React.useEffect(() => {
    if (!rmf) {
      console.error('Unable to get latest robot information, fleets API unavailable');
      return;
    }

    const refreshRobotTable = async () => {
      const fleets = (await rmf.fleetsApi.getFleetsFleetsGet()).data;
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
            ? (await rmf.tasksApi.queryTaskStatesTasksGet(taskIds.join(','))).data.reduce(
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
              ? Object.entries(fleet.robots).map<RobotTableData>(([name, robot]) => ({
                  fleet: fleet.name || '',
                  name,
                  battery: robot.battery && +robot.battery.toFixed(2),
                  status: robot.status,
                  estFinishTime:
                    robot.task_id && tasks[robot.task_id]
                      ? tasks[robot.task_id].unix_millis_finish_time
                      : undefined,
                  lastUpdateTime: robot.unix_millis_time ? robot.unix_millis_time : undefined,
                  level: robot.location?.map || 'N/A',
                  commission: robot.commission,
                }))
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
  }, [rmf]);

  return (
    <TableContainer>
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
});
