import { ApiServerModelsTortoiseModelsAlertsAlertLeaf, RobotState, TaskState } from 'api-client';
import React from 'react';
import { RmfAppContext } from './rmf-app';
import { TaskAlertHandler } from './tasks/task-alert';

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

  // const robotAlertKey: Partial<typeof robotAlerts> = { ...robotAlerts };
  // const removeRobotAlert = (id: string) => {
  //   delete robotAlertKey[id];
  // };

  return (
    <>
      <TaskAlertHandler alerts={Object.values(taskAlerts)} removeAlert={removeTaskAlert} />
      {/* <RobotAlertHandler alerts={Object.values(robotAlerts)}/> */}
    </>
  );
});
