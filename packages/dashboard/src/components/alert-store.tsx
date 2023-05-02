import { ApiServerModelsTortoiseModelsAlertsAlertLeaf, RobotState, TaskState } from 'api-client';
import React from 'react';
import { RmfAppContext } from './rmf-app';
import { TaskAlertHandler } from './tasks/task-alert';
import { RobotAlertHandler } from './robots/robot-alert';

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

// This needs to match the enums provided for the Alert model, as it is not
// provided via the api-client since tortoise's pydantic_model_creator is used.
enum AlertCategory {
  Default = 'default',
  Task = 'task',
  Fleet = 'fleet',
  Robot = 'robot',
}

export const AlertStore = React.memo(() => {
  const rmf = React.useContext(RmfAppContext);
  const [taskAlerts, setTaskAlerts] = React.useState<Record<string, Alert>>({});
  const [robotAlerts, setRobotAlerts] = React.useState<Record<string, Alert>>({});

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = rmf.alertObsStore.subscribe(async (alert) => {
      switch (alert.category) {
        case AlertCategory.Task:
          setTaskAlerts((prev) => ({ ...prev, [alert.id]: alert }));
          break;
        case AlertCategory.Fleet:
        case AlertCategory.Robot:
          setRobotAlerts((prev) => ({ ...prev, [alert.id]: alert }));
          break;
        default:
        // do nothing in default case
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
      <RobotAlertHandler alerts={Object.values(robotAlerts)} removeAlert={removeRobotAlert} />
    </>
  );
});
