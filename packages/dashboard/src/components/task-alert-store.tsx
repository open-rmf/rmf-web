import { RobotState, TaskState } from 'api-client';
import React from 'react';
import { RmfAppContext } from './rmf-app';
import { AlertComponent } from './task-alert';

export interface RobotWithTask {
  task?: TaskState;
  robot?: RobotState;
}

export const TaskAlertStore = React.memo(() => {
  const rmf = React.useContext(RmfAppContext);

  const [fleets, setFleets] = React.useState<string[]>([]);
  const [robotsWithTasks, setRobotsWithTasks] = React.useState<Record<string, RobotWithTask[]>>({});

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
      setRobotsWithTasks({});
    });
    return () => sub.unsubscribe();
  }, [rmf]);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const subs = fleets.map((f) =>
      rmf.getFleetStateObs(f).subscribe(async (fleet) => {
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

        setRobotsWithTasks((prev) => {
          if (!fleet.name) {
            return prev;
          }
          return {
            ...prev,
            [fleet.name]: fleet.robots
              ? Object.entries(fleet.robots).map<RobotWithTask>(([name, robot]) =>
                  robot.task_id
                    ? {
                        task: tasks[robot.task_id],
                        robot: robot,
                      }
                    : {},
                )
              : [],
          };
        });
      }),
    );
    return () => subs.forEach((sub) => sub.unsubscribe());
  }, [rmf, fleets]);

  return (
    <>
      <AlertComponent
        robots={Object.values(robotsWithTasks)
          .flatMap((r) => r)
          .filter((element) => {
            if (Object.keys(element).length !== 0) {
              return true;
            }
            return false;
          })}
      />
    </>
  );
});
