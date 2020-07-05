import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import { RobotLoopForm } from './robot-item-form';
import fakePlaces from '../mock/data/places';
import { v4 as uuidv4 } from 'uuid';
import { SpotlightValue } from './spotlight-value';
import { Typography } from '@material-ui/core';

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

export interface CommandsPanelProps {
  fleets: readonly RomiCore.FleetState[];
  spotlight?: Readonly<SpotlightValue<string>>;
  transport?: Readonly<RomiCore.Transport>;
}

export default function CommandsPanel(props: CommandsPanelProps): React.ReactElement {
  const { fleets, spotlight, transport } = props;
  const commandRefs = React.useRef<Record<string, HTMLElement | null>>({});
  const [expanded, setExpanded] = React.useState<Readonly<Record<string, boolean>>>({});
  const listOfPlaces = fakePlaces()['magni'];

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
    const ref = commandRefs.current[spotlight.value];
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
      <Typography variant="h6">Task Requests</Typography>
      {fleets.flatMap(fleet => (
        <RobotLoopForm
          requestLoop={handleRequestLoop}
          fleetName={fleet.name}
          listOfPlaces={listOfPlaces}
        />
      ))}
    </React.Fragment>
  );
}
