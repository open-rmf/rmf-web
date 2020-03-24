/**
 * TODO: Show indicator why door controls are disabled.
 */

import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import DoorItem from './door-item';
import { SpotlightValue } from './spotlight-expansion-panel';

export interface DoorsPanelProps {
  doors: readonly RomiCore.Door[];
  doorStates: Readonly<Record<string, RomiCore.DoorState | undefined>>;
  transport?: Readonly<RomiCore.Transport>;
  spotlight?: SpotlightValue<string>;
}

export default function DoorsPanel(props: DoorsPanelProps): JSX.Element {
  const { transport, spotlight } = props;
  const doorRefs = React.useRef<Record<string, HTMLElement | null>>({});
  const [doorRequestPub, setDoorRequestPub] = React.useState<DoorRequestPublisher | null>(null);

  function requestDoor(door: RomiCore.Door, mode: number): void {
    doorRequestPub?.publish({
      door_name: door.name,
      requested_mode: { value: mode },
      requester_id: props.transport!.name,
      request_time: RomiCore.toRosTime(new Date()),
    });
  }

  React.useEffect(() => {
    setDoorRequestPub(transport ? transport.createPublisher(RomiCore.doorRequests) : null);
  }, [transport]);

  React.useEffect(() => {
    if (!spotlight) {
      return;
    }
    const doorRef = doorRefs.current[spotlight.value];
    if (!doorRef) {
      return;
    }

    doorRef.scrollIntoView({ behavior: 'smooth' });
  }, [spotlight]);

  const listItems = props.doors.map(door => {
    const doorState = props.doorStates[door.name];
    return (
      <DoorItem
        key={door.name}
        ref={ref => (doorRefs.current[door.name] = ref)}
        door={door}
        doorState={doorState}
        enableControls={Boolean(transport)}
        onOpenClick={() => requestDoor(door, RomiCore.DoorMode.MODE_OPEN)}
        onCloseClick={() => requestDoor(door, RomiCore.DoorMode.MODE_CLOSED)}
      />
    );
  });

  return <React.Fragment>{listItems}</React.Fragment>;
}

type DoorRequestPublisher = RomiCore.Publisher<RomiCore.DoorRequest>;
