import { SpotlightValue } from './spotlight-value';
import { v4 as uuidv4 } from 'uuid';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import RobotItem from './robot-item';

/**
    * task_id is intended to be a pseudo-random string generated by the caller which can be used to 
    * identify this task as it moves between the queues to completion (or failure).

    * robot_type can be used to specify a particular robot fleet for this request

    * num_loops The number of times the robot should loop between the specified points. 

    * start_name The name of the waypoint where the robot should begin its loop. If the robot is 
    * not already at this point, it will begin the task by moving there.

    * finish_name The name of the waypoint where the robot should end its looping. The robot will
    * visit this waypoint num_loops times and then stop here on the last visit.
   */
export function requestLoop(
  loopRequestPub: RomiCore.Publisher<RomiCore.Loop> | null,
  robotName: string,
  numLoops: number,
  startLocationPoint: string,
  endLocationPoint: string,
) {
  loopRequestPub?.publish({
    finish_name: endLocationPoint,
    num_loops: numLoops,
    robot_type: robotName,
    start_name: startLocationPoint,
    task_id: uuidv4(),
  });
}

export interface RobotsPanelProps {
  fleets: readonly RomiCore.FleetState[];
  spotlight?: Readonly<SpotlightValue<string>>;
  transport?: Readonly<RomiCore.Transport>;
  onRobotClick?(robot: RomiCore.RobotState): void;
}

export default function RobotsPanel(props: RobotsPanelProps): React.ReactElement {
  const { fleets, spotlight, onRobotClick, transport } = props;
  const robotRefs = React.useRef<Record<string, HTMLElement | null>>({});
  const [expanded, setExpanded] = React.useState<Readonly<Record<string, boolean>>>({});

  const loopRequestPub = React.useMemo(
    () => (transport ? transport.createPublisher(RomiCore.loopRequests) : null),
    [transport],
  );

  const handleRequestLoop = (
    fleetName: string,
    numLoops: number,
    startLocationPoint: string,
    endLocationPoint: string,
  ) => {
    requestLoop(loopRequestPub, fleetName, numLoops, startLocationPoint, endLocationPoint);
  };

  React.useEffect(() => {
    if (!spotlight) {
      return;
    }
    const ref = robotRefs.current[spotlight.value];
    if (!ref) {
      return;
    }
    setExpanded(prev => ({
      ...prev,
      [spotlight.value]: true,
    }));
    ref.scrollIntoView({ behavior: 'smooth' });
  }, [spotlight]);

  return (
    <React.Fragment>
      {fleets.flatMap(fleet =>
        fleet.robots.map(robot => (
          <RobotItem
            key={robot.name}
            ref={ref => (robotRefs.current[robot.name] = ref)}
            requestLoop={handleRequestLoop}
            fleetName={fleet.name}
            robot={robot}
            onClick={() => onRobotClick && onRobotClick(robot)}
            expanded={Boolean(expanded[robot.name])}
            onChange={(_, newExpanded) =>
              setExpanded(prev => ({
                ...prev,
                [robot.name]: newExpanded,
              }))
            }
            TransitionProps={{ unmountOnExit: true }}
          />
        )),
      )}
    </React.Fragment>
  );
}
