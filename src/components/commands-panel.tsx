import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import { LoopForm } from './loop-form';
import { v4 as uuidv4 } from 'uuid';
import { SpotlightValue } from './spotlight-value';

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

  const allFleets = fleets.flatMap(fleet => fleet.name);
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
    ref.scrollIntoView({ behavior: 'smooth' });
  }, [spotlight]);

  return (
    <React.Fragment>
      <LoopForm requestLoop={handleRequestLoop} fleets={allFleets} />
    </React.Fragment>
  );
}
