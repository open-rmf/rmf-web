import { TableContainer } from '@mui/material';
import { TaskState } from 'api-client';
import React from 'react';
import { RobotTable, RobotTableData } from 'react-components';
import { AppEvents } from '../app-events';
import { createMicroApp } from '../micro-app';
import { RmfAppContext } from '../rmf-app';

export const RobotsApp = createMicroApp('Robots', () => {
  const rmf = React.useContext(RmfAppContext);
  const [robots, setRobots] = React.useState<RobotTableData[]>([]);
  React.useEffect(() => {
    if (!rmf) {
      return;
    }

    // TODO: support auto refresh
    rmf.fleetsObs.subscribe(async (fleets) => {
      // fetch active tasks
      const taskIds = fleets.flatMap(
        (fleet) => fleet.robots && Object.values(fleet.robots).map((robot) => robot.task_id),
      );
      const tasks = (await rmf.tasksApi.queryTaskStatesTasksGet(taskIds.join(','))).data.reduce(
        (acc, task) => {
          acc[task.booking.id] = task;
          return acc;
        },
        {} as Record<string, TaskState>,
      );

      setRobots(
        fleets.flatMap((fleet) =>
          fleet.robots
            ? Object.entries(fleet.robots).map<RobotTableData>(([name, robot]) => ({
                fleet: fleet.name || '',
                name,
                battery: robot.battery,
                status: robot.status,
                estFinishTime:
                  robot.task_id && tasks[robot.task_id]
                    ? tasks[robot.task_id].estimate_millis
                    : undefined,
              }))
            : [],
        ),
      );
    });
  });

  return (
    <TableContainer>
      <RobotTable
        robots={robots}
        onRobotClick={(_ev, robot) => {
          AppEvents.robotSelect.next(['', robot.name]);
        }}
      />
    </TableContainer>
  );
});
