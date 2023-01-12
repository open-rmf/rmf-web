import React from 'react';
import { AlertProps, AlertToDisplay } from '../alert-store';
import { Status } from 'api-client';
import { RmfAppContext } from '../rmf-app';
import { Subscription } from 'rxjs';
import { AlertDialog } from '../alert-dialog-component';

export function TaskAlertComponent({ robots }: AlertProps): JSX.Element {
  const rmf = React.useContext(RmfAppContext);
  const [alertToDisplay, setAlertToDisplay] = React.useState<AlertToDisplay[]>([]);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }

    const subs: Subscription[] = [];

    robots.map((r) => {
      if (r.robot?.task_id) {
        return subs.push(
          rmf.getTaskStateObs(r.robot.task_id).subscribe((task) => {
            if (task.status === Status.Completed || task.status === Status.Failed) {
              setAlertToDisplay((prev) => [...prev, { show: true, task: task, robot: r.robot }]);
            }
          }),
        );
      }
      return [];
    });

    return () => subs.forEach((s) => s.unsubscribe());
  }, [rmf, robots]);

  return (
    <>
      {alertToDisplay.map((r) =>
        r.show ? (
          <AlertDialog
            key={r.robot.name}
            current={r}
            setValue={setAlertToDisplay}
            robotAlert={false}
          />
        ) : null,
      )}
    </>
  );
}
