import React from 'react';
import { RobotState, Status2 } from 'api-client';
import { RmfAppContext } from '../rmf-app';
import { AlertProps, AlertToDisplay, RobotWithTask } from '../task-alert-store';
import { AlertDialog } from '../alert-dialog-component';

const statusToAlert = (robot: RobotState) => {
  switch (robot.status) {
    case Status2.Working:
    case Status2.Idle:
    case Status2.Error:
    case Status2.Offline:
      return true;
    default:
      return false;
  }
};

export function RobotAlertComponent({ robots }: AlertProps): JSX.Element {
  const rmf = React.useContext(RmfAppContext);
  const [robotsInStorage, setRobotsInStorage] = React.useState<AlertToDisplay[]>([]);

  // Set all robots in localstorage
  React.useEffect(() => {
    if (!robotsInStorage.length) {
      robots.map((r) =>
        setRobotsInStorage((prev) => [...prev, { show: true, task: r.task, robot: r.robot }]),
      );
    }
  }, [robotsInStorage, robots]);

  // Set robots from localStorage into local state in the first rendering
  React.useEffect(() => {
    const data = window.localStorage.getItem('robots');
    if (data !== null) {
      setRobotsInStorage(JSON.parse(data));
    }
  }, []);

  React.useEffect(() => {
    localStorage.setItem('robots', JSON.stringify(robotsInStorage));
  }, [robotsInStorage]);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }

    const localItems = localStorage.getItem('robots');
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
        return setRobotsInStorage((prev) =>
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
      {robotsInStorage.map((r) =>
        statusToAlert(r.robot) && r.show ? (
          <AlertDialog
            key={r.robot.name}
            current={r}
            setValue={setRobotsInStorage}
            robotAlert={true}
          />
        ) : null,
      )}
    </>
  );
}
