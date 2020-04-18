import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import RobotItem from './robot-item';
import { SpotlightValue } from './spotlight-value';

export interface RobotsPanelProps {
  fleets: readonly RomiCore.FleetState[];
  spotlight?: Readonly<SpotlightValue<string>>;
  onRobotClick?(robot: RomiCore.RobotState): void;
}

export default function RobotsPanel(props: RobotsPanelProps): React.ReactElement {
  const { fleets, spotlight, onRobotClick } = props;
  const robotRefs = React.useRef<Record<string, HTMLElement | null>>({});
  const [expanded, setExpanded] = React.useState<Readonly<Record<string, boolean>>>({});

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
