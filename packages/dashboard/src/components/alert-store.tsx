import { ApiServerModelsTortoiseModelsAlertsAlertLeaf, RobotState, TaskState } from 'api-client';
import React from 'react';
import { RmfAppContext } from './rmf-app';
// import { RobotAlertComponent } from './robots/robot-alert';
import { TaskAlertHandler } from './tasks/task-alert';
// import { TaskAlert } from 'react-components';

type Alert = ApiServerModelsTortoiseModelsAlertsAlertLeaf;

export interface RobotWithTask {
  task?: TaskState;
  robot: RobotState;
}

export interface AlertStoreProps {
  robots: RobotWithTask[];
}

export interface AlertToDisplay extends RobotWithTask {
  show: boolean;
}

const returnOnlyRobotsWithTask = (value: Record<string, RobotWithTask[]>) => {
  return Object.values(value)
    .flatMap((r) => r)
    .filter((element) => {
      if (element.task) {
        return true;
      }
      return false;
    });
};

export const AlertStore = React.memo(() => {
  const rmf = React.useContext(RmfAppContext);
  const [taskAlerts, setTaskAlerts] = React.useState<Record<string, Alert>>({});
  const [robotAlerts, setRobotAlerts] = React.useState<Record<string, Alert>>({});

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = rmf.alertObsStore.subscribe(async (alert) => {
      if (alert.category === 'task') {
        setTaskAlerts((prev) => {
          return {
            ...prev,
            [alert.id]: alert,
          };
        });
      } else if (alert.category === 'fleet' || alert.category === 'robot') {
        setRobotAlerts((prev) => {
          return {
            ...prev,
            [alert.id]: alert,
          };
        });
      }
    });
    return () => sub.unsubscribe();
  }, [rmf]);

  const taskAlertKey: Partial<typeof taskAlerts> = { ...taskAlerts };
  const removeTaskAlert = (id: string) => {
    delete taskAlertKey[id];
  };

  const robotAlertKey: Partial<typeof robotAlerts> = { ...robotAlerts };
  const removeRobotAlert = (id: string) => {
    delete robotAlertKey[id];
  };

  return (
    <>
      <TaskAlertHandler alerts={Object.values(taskAlerts)} removeAlert={removeTaskAlert} />
      {/* <RobotAlertHandler alerts={Object.values(robotAlerts)}/> */}
    </>
    // alert && alert.category === "task" ? (
    //   <>
    //     <TaskAlert
    //       id={alert.id}
    //     />
    //   </>
    // ) : (
    //   null
    // )
  );

  // const [fleets, setFleets] = React.useState<string[]>([]);
  // const [robotsWithTasks, setRobotsWithTasks] = React.useState<Record<string, RobotWithTask[]>>({});

  // React.useEffect(() => {
  //   if (!rmf) {
  //     return;
  //   }

  //   const sub = rmf.fleetsObs.subscribe((fleets) => {
  //     setFleets(
  //       fleets.reduce<string[]>((acc, f) => {
  //         if (f.name) {
  //           acc.push(f.name);
  //         }
  //         return acc;
  //       }, []),
  //     );
  //   });
  //   return () => sub.unsubscribe();
  // }, [rmf]);

  // React.useEffect(() => {
  //   if (!rmf) {
  //     return;
  //   }
  //   const subs = fleets.map((f) =>
  //     rmf.getFleetStateObs(f).subscribe(async (fleet) => {
  //       const taskIds = fleet.robots
  //         ? Object.values(fleet.robots).reduce<string[]>((acc, robot) => {
  //             if (robot.task_id) {
  //               acc.push(robot.task_id);
  //             }
  //             return acc;
  //           }, [])
  //         : [];

  //       const tasks =
  //         taskIds.length > 0
  //           ? (await rmf.tasksApi.queryTaskStatesTasksGet(taskIds.join(','))).data.reduce(
  //               (acc, task) => {
  //                 acc[task.booking.id] = task;
  //                 return acc;
  //               },
  //               {} as Record<string, TaskState>,
  //             )
  //           : {};

  //       setRobotsWithTasks((prev) => {
  //         if (!fleet.name) {
  //           return prev;
  //         }
  //         return {
  //           ...prev,
  //           [fleet.name]: fleet.robots
  //             ? Object.entries(fleet.robots).map<RobotWithTask>(([name, robot]) => ({
  //                 task: robot.task_id ? tasks[robot.task_id] : undefined,
  //                 robot: robot,
  //               }))
  //             : [],
  //         };
  //       });
  //     }),
  //   );
  //   return () => subs.forEach((sub) => sub.unsubscribe());
  // }, [rmf, fleets]);

  // return (
  //   <>
  //     <TaskAlertComponent robots={returnOnlyRobotsWithTask(robotsWithTasks)} />
  //     <RobotAlertComponent robots={Object.values(robotsWithTasks).flatMap((r) => r)} />
  //   </>
  // );
});
