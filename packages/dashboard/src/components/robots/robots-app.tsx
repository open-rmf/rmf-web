import { TableContainer } from '@mui/material';
import { TaskState } from 'api-client';
import React from 'react';
import { RobotTable, RobotTableData } from 'react-components';
import { AppEvents } from '../app-events';
import { createMicroApp } from '../micro-app';
import { RmfAppContext } from '../rmf-app';

export const RobotsApp = createMicroApp('Robots', () => {
  const rmf = React.useContext(RmfAppContext);

  const [fleets, setFleets] = React.useState<string[]>([]);
  const [robots, setRobots] = React.useState<Record<string, RobotTableData[]>>({});
  React.useEffect(() => {
    if (!rmf) {
      return;
    }

    const sub = rmf.fleetsObs.subscribe((fleets) => {
      setFleets(
        fleets.reduce<string[]>((acc, f) => {
          if (f.name) {
            acc.push(f.name);
          }
          return acc;
        }, []),
      );
      setRobots({});
    });
    return () => sub.unsubscribe();
  }, [rmf]);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const subs = fleets.map((f) =>
      rmf.getFleetStateObs(f).subscribe(async (fleet) => {
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
                  battery: robot.battery,
                  status: robot.status,
                  estFinishTime:
                    robot.task_id && tasks[robot.task_id]
                      ? tasks[robot.task_id].unix_millis_finish_time
                      : undefined,
                }))
              : [],
          };
        });
      }),
    );
    return () => subs.forEach((sub) => sub.unsubscribe());
  }, [rmf, fleets]);

  return (
    <TableContainer>
      <RobotTable
        robots={Object.values(robots).flatMap((r) => r)}
        onRobotClick={(_ev, robot) => {
          AppEvents.robotSelect.next([robot.fleet, robot.name]);
        }}
      />
    </TableContainer>
  );
});
