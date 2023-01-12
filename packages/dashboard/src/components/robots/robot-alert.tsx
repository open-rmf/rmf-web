import React from 'react';
import { Status2 } from 'api-client';
import { RmfAppContext } from '../rmf-app';
import { RobotWithTask } from '../task-alert-store';
import { AlertDialog } from '../dialog-component';

export interface AlertProps {
  robots: RobotWithTask[];
}

const statusToAlert = (r: AlertToDisplay) => {
  switch (r.robot?.status) {
    case Status2.Working:
    case Status2.Idle:
    case Status2.Error:
    case Status2.Offline:
      return true;
    default:
      return false;
  }
};

interface AlertToDisplay extends RobotWithTask {
  show: boolean;
}

export function RobotAlertComponent({ robots }: AlertProps): JSX.Element {
  const rmf = React.useContext(RmfAppContext);
  const [robotsInStorage, setRobotsInStorage] = React.useState<AlertToDisplay[]>([]);

  React.useEffect(() => {
    if (!robotsInStorage.length) {
      robots.map((r) =>
        setRobotsInStorage((prev) => [...prev, { show: true, task: r.task, robot: r.robot }]),
      );
    }
  }, [robotsInStorage, robots]);

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

    const local = localStorage.getItem('robots');
    if (!local) {
      return;
    }

    const robotLocal: RobotWithTask[] = JSON.parse(local);

    robots.map((r) =>
      robotLocal.map((i) =>
        r.robot?.name === i.robot?.name
          ? r.robot?.status !== i.robot?.status
            ? setRobotsInStorage((current) =>
                current.map((obj) => {
                  if (obj.robot?.name === i.robot?.name) {
                    return { ...obj, robot: r.robot, task: r.task, show: true };
                  }
                  return obj;
                }),
              )
            : []
          : [],
      ),
    );
  }, [rmf, robots]);

  return (
    <>
      {robotsInStorage.map((r) =>
        statusToAlert(r) && r.show ? (
          <AlertDialog key={r.robot?.name} current={r} setValue={setRobotsInStorage} />
        ) : null,
      )}
    </>
  );
}
