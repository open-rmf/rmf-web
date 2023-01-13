import React from 'react';
import { AlertProps, AlertToDisplay } from '../alert-store';
import { Status, TaskState } from 'api-client';
import { RmfAppContext } from '../rmf-app';
import { Subscription } from 'rxjs';
import { AlertDialog, AlertContent } from '../alert-dialog-component';
import { base } from 'react-components';

const showMessage = (task: TaskState | undefined) => {
  if (!task) {
    return 'No message';
  }

  switch (task.status) {
    case Status.Failed:
      return `${task.dispatch?.status} 
                ${task.dispatch?.errors?.map((e) => e.message)}`;

    case Status.Completed:
      return 'Task completed!';

    default:
      return 'No message';
  }
};

const setTaskDialogColor = (taskStatus: Status | undefined) => {
  if (!taskStatus) {
    return base.palette.background.default;
  }

  switch (taskStatus) {
    case Status.Failed:
      return base.palette.error.dark;

    case Status.Completed:
      return base.palette.success.dark;

    default:
      return base.palette.background.default;
  }
};

const buildDialogContent = (alertToDisplay: AlertToDisplay): AlertContent[] => {
  return [
    {
      title: 'Task',
      value: alertToDisplay.task ? alertToDisplay.task.booking.id : '',
    },
    {
      title: 'Location',
      value: alertToDisplay.robot.location ? alertToDisplay.robot.location.map : '',
    },
    {
      title: 'Message',
      value: showMessage(alertToDisplay.task),
    },
  ];
};

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
            alertToDisplay={r}
            setValue={setAlertToDisplay}
            dialogTitle={'Task State'}
            progress={r.robot.battery ? r.robot.battery : -1}
            alertContents={buildDialogContent(r)}
            backgroundColor={setTaskDialogColor(r.task?.status)}
          />
        ) : null,
      )}
    </>
  );
}
