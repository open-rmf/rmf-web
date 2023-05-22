import {
  ApiServerModelsTortoiseModelsAlertsAlertLeaf as Alert,
  RobotState,
  TaskState,
} from 'api-client';
import { AppEvents } from './app-events';
import React from 'react';
import { RmfAppContext } from './rmf-app';
import { Subscription } from 'rxjs';
import { TaskAlertDialog } from './tasks/task-alert';
import { RobotAlertDialog } from './robots/robot-alert';

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
  const [refreshAlertCount, setRefreshAlertCount] = React.useState(0);

  const categorizeAndPushAlerts = (alert: Alert) => {
    // We check if an existing alert has been acknowledged, remove it before
    // adding the acknowledged alert.
    switch (alert.category) {
      case AlertCategory.Task:
        setTaskAlerts((prev) => {
          const filteredTaskAlerts: Record<string, Alert> = {};
          for (let key in prev) {
            if (key !== alert.original_id) {
              filteredTaskAlerts[key] = prev[key];
            }
          }
          filteredTaskAlerts[alert.id] = alert;
          return filteredTaskAlerts;
        });
        break;
      case AlertCategory.Fleet:
      case AlertCategory.Robot:
        setRobotAlerts((prev) => {
          const filteredRobotAlerts: Record<string, Alert> = {};
          for (let key in prev) {
            if (key !== alert.original_id) {
              filteredRobotAlerts[key] = prev[key];
            }
          }
          filteredRobotAlerts[alert.id] = alert;
          return filteredRobotAlerts;
        });
        break;
      default:
      // do nothing in default case
    }
  };

  React.useEffect(() => {
    const subs: Subscription[] = [];
    subs.push(
      AppEvents.alertListOpenedAlert.subscribe((alert) => {
        if (alert) {
          categorizeAndPushAlerts(alert);
        }
      }),
    );
    subs.push(
      AppEvents.refreshAlertCount.subscribe((count) => {
        setRefreshAlertCount(count);
      }),
    );
    return () => subs.forEach((s) => s.unsubscribe());
  }, []);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = rmf.alertObsStore.subscribe(async (alert) => {
      categorizeAndPushAlerts(alert);
      AppEvents.refreshAlertCount.next(refreshAlertCount + 1);
    });
    return () => sub.unsubscribe();
  }, [rmf, refreshAlertCount]);

  const removeTaskAlert = (id: string) => {
    const filteredTaskAlerts: Record<string, Alert> = {};
    for (let key in taskAlerts) {
      if (key !== id) {
        filteredTaskAlerts[key] = taskAlerts[key];
      }
    }
    setTaskAlerts(filteredTaskAlerts);
  };

  const removeRobotAlert = (id: string) => {
    const filteredRobotAlerts: Record<string, Alert> = {};
    for (let key in robotAlerts) {
      if (key !== id) {
        filteredRobotAlerts[key] = robotAlerts[key];
      }
    }
    setRobotAlerts(filteredRobotAlerts);
  };

  return (
    <>
      {Object.values(taskAlerts).map((alert) => {
        const removeThisAlert = () => {
          removeTaskAlert(alert.id);
        };
        return <TaskAlertDialog key={alert.id} alert={alert} removeAlert={removeThisAlert} />;
      })}
      {Object.values(robotAlerts).map((alert) => {
        const removeThisAlert = () => {
          removeRobotAlert(alert.id);
        };
        return <RobotAlertDialog key={alert.id} alert={alert} removeAlert={removeThisAlert} />;
      })}
    </>
  );
});
