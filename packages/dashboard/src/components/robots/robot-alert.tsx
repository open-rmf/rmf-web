import React from 'react';
import { RobotState, Status2 } from 'api-client';
import { RmfAppContext } from '../rmf-app';
import { AlertProps, AlertToDisplay, RobotWithTask } from '../alert-store';
import { AlertDialog, AlertContent } from '../alert-dialog-component';
import { base } from 'react-components';

const statusToAlert = (robot: RobotState) => {
  switch (robot.status) {
    case Status2.Error:
    case Status2.Offline:
      return true;
    default:
      return false;
  }
};

const setRobotDialogColor = (robotStatus: Status2 | undefined) => {
  if (!robotStatus) {
    return base.palette.background.default;
  }

  switch (robotStatus) {
    case Status2.Error:
      return base.palette.error.main;

    case Status2.Offline:
      return base.palette.warning.main;

    default:
      return base.palette.background.default;
  }
};

const showMessage = (robot: RobotState) => {
  switch (robot.status) {
    case Status2.Error:
      if (robot.issues) {
        robot.issues.map((i) => {
          if (!i.detail) {
            return 'No Detail';
          }
          if (Array.isArray(i.detail)) {
            i.detail.map(
              (d) => ` * ${d} 
                `,
            );
          }
          return i.detail;
        });
      }
      return 'Robot changed its state to Error.';

    case Status2.Offline:
      return 'Robot changed its state to offline';

    default:
      return 'No message';
  }
};

const buildDialogContent = (robot: RobotState): AlertContent[] => {
  return [
    {
      title: 'Robot Name',
      value: robot.name ? robot.name : 'No name',
    },
    {
      title: 'Location',
      value: robot.location ? robot.location.map : '',
    },
    {
      title: 'Message',
      value: showMessage(robot),
    },
  ];
};

export function RobotAlertComponent({ robots }: AlertProps): JSX.Element {
  const rmf = React.useContext(RmfAppContext);
  const [alertsToDisplay, setAlertsToDisplay] = React.useState<AlertToDisplay[]>([]);

  // Set all robots in localstorage
  React.useEffect(() => {
    if (!alertsToDisplay.length) {
      robots.map((r) =>
        setAlertsToDisplay((prev) => [...prev, { show: true, task: r.task, robot: r.robot }]),
      );
    }
  }, [alertsToDisplay, robots]);

  // Set robots from localStorage into local state in the first rendering
  React.useEffect(() => {
    const data = window.localStorage.getItem('robots_with_tasks');
    if (data !== null) {
      setAlertsToDisplay(JSON.parse(data));
    }
  }, []);

  React.useEffect(() => {
    localStorage.setItem('robots_with_tasks', JSON.stringify(alertsToDisplay));
  }, [alertsToDisplay]);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }

    const localItems = localStorage.getItem('robots_with_tasks');
    if (!localItems) {
      return;
    }

    const robotsLocal: RobotWithTask[] = JSON.parse(localItems);

    /**
     * Loop through the robots array and compare with the local storage element.
     * If the state of the robot changes we have to set "show" property to "true" so that we can show the
     * alert after if the state meets with this "statusToAlert" function
     */
    robots.map((r) => {
      const robotItem = robotsLocal.find((rl) => r.robot?.name === rl.robot?.name);

      if (!robotItem) {
        return [];
      }

      if (r.robot?.status !== robotItem.robot?.status) {
        return setAlertsToDisplay((prev) =>
          prev.map((alertToDisplay) =>
            alertToDisplay.robot?.name === robotItem.robot?.name
              ? { ...alertToDisplay, robot: r.robot, task: r.task, show: true }
              : alertToDisplay,
          ),
        );
      }
      return [];
    });
  }, [rmf, robots]);

  return (
    <>
      {alertsToDisplay.map((r) =>
        statusToAlert(r.robot) && r.show ? (
          <AlertDialog
            key={r.robot.name}
            stopShowing={() =>
              setAlertsToDisplay((prev) =>
                prev.map((obj) => {
                  if (obj.robot.name === r.robot.name) {
                    return { ...obj, show: false };
                  }
                  return obj;
                }),
              )
            }
            dialogTitle={'Robot State'}
            progress={r.robot.battery ? r.robot.battery : -1}
            alertContents={buildDialogContent(r.robot)}
            backgroundColor={setRobotDialogColor(r.robot.status)}
            show={r.show}
          />
        ) : null,
      )}
    </>
  );
}
