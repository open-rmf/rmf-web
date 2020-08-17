import React from 'react';

import { RobotItem } from '../../components/robot-item';
import { RobotsPanelProps } from '../../components/robots-panel';

export default function RobotButton(props: RobotsPanelProps) {
  const { fleets } = props;
  const robotRefs = React.useRef<Record<string, HTMLElement | null>>({});

  return (
    <React.Fragment>
      {fleets.flatMap(fleet => {
        return fleet.robots.map(robot => {
          return (
            <RobotItem
              key={robot.name}
              ref={ref => (robotRefs.current[robot.name] = ref)}
              fleetName={fleet.name}
              robot={robot}
            />
          );
        });
      })}
    </React.Fragment>
  );
}
