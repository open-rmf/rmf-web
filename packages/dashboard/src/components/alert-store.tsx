import { ApiServerModelsTortoiseModelsAlertsAlertLeaf as Alert } from 'api-client';
import { AppEvents } from './app-events';
import React from 'react';
import { RmfAppContext } from './rmf-app';
import { Subscription } from 'rxjs';
import { TaskAlertDialog } from './tasks/task-alert';

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

  // const filterAndPushAlerts = (alert: Alert) => {
  //   // We check if an existing alert has been acknowledged, remove it before
  //   // adding the acknowledged alert.
  //   if (alert.category === AlertCategory.Task) {
  //     setTaskAlerts((prev) => {
  //       const filteredTaskAlerts = Object.fromEntries(
  //         Object.entries(prev).filter(([key]) => key !== alert.original_id),
  //       );
  //       filteredTaskAlerts[alert.id] = alert;
  //       return filteredTaskAlerts;
  //     });
  //   }
  // };

  React.useEffect(() => {
    const subs: Subscription[] = [];
    subs.push(
      AppEvents.alertListOpenedAlert.subscribe((alert) => {
        if (alert) {
          setTaskAlerts((prev) => {
            // const updatedAlerts = prev;
            // updatedAlerts[alert.id] = alert;
            // return updatedAlerts;

            const filteredTaskAlerts = Object.fromEntries(
              Object.entries(prev).filter(([key]) => key !== alert.original_id),
            );
            filteredTaskAlerts[alert.id] = alert;
            return filteredTaskAlerts;
          });
        }
      }),
    );
    return () => subs.forEach((s) => s.unsubscribe());
  }, []);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = rmf.alertObsStore.subscribe(async (alert) => {
      setTaskAlerts((prev) => {
        if (Object.keys(prev).includes(alert.original_id)) {
          const filteredTaskAlerts = Object.fromEntries(
            Object.entries(prev).filter(([key]) => key !== alert.original_id),
          );
          return filteredTaskAlerts;
        }

        const updatedAlerts = prev;
        updatedAlerts[alert.id] = alert;
        return updatedAlerts;
      });
      AppEvents.refreshAlert.next();
    });
    return () => sub.unsubscribe();
  }, [rmf]);

  const removeTaskAlert = (id: string) => {
    const filteredTaskAlerts = Object.fromEntries(
      Object.entries(taskAlerts).filter(([key]) => key !== id),
    );
    setTaskAlerts(filteredTaskAlerts);
  };

  return (
    <>
      {Object.values(taskAlerts).map((alert) => {
        const removeThisAlert = () => {
          removeTaskAlert(alert.id);
        };
        return <TaskAlertDialog key={alert.id} alert={alert} removeAlert={removeThisAlert} />;
      })}
    </>
  );
});
