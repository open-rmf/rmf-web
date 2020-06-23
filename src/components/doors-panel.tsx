import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import DoorItem from './door-item';
import { SpotlightValue } from './spotlight-value';

export interface DoorsPanelProps {
  doors: readonly RomiCore.Door[];
  doorStates: Readonly<Record<string, RomiCore.DoorState>>;
  transport?: Readonly<RomiCore.Transport>;
  spotlight?: Readonly<SpotlightValue<string>>;
  onDoorClick?(door: RomiCore.Door): void;
}

export default function DoorsPanel(props: DoorsPanelProps): JSX.Element {
  const { transport, spotlight, onDoorClick } = props;
  const doorRefs = React.useRef<Record<string, HTMLElement | null>>({});
  const [expanded, setExpanded] = React.useState<Record<string, boolean>>({});
  const doorRequestPub = React.useMemo(
    () => (transport ? transport.createPublisher(RomiCore.adapterDoorRequests) : null),
    [transport],
  );

  function requestDoor(door: RomiCore.Door, mode: number): void {
    doorRequestPub?.publish({
      door_name: door.name,
      requested_mode: { value: mode },
      requester_id: transport!.name,
      request_time: RomiCore.toRosTime(new Date()),
    });
  }

  React.useEffect(() => {
    if (!spotlight) {
      return;
    }
    const ref = doorRefs.current[spotlight.value];
    if (!ref) {
      return;
    }
    setExpanded(prev => ({
      ...prev,
      [spotlight.value]: true,
    }));
    ref.scrollIntoView({ behavior: 'smooth' });
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
        onClick={() => onDoorClick && onDoorClick(door)}
        expanded={Boolean(expanded[door.name])}
        onChange={(_, newExpanded) =>
          setExpanded(prev => ({
            ...prev,
            [door.name]: newExpanded,
          }))
        }
      />
    );
  });

  return <React.Fragment>{listItems}</React.Fragment>;
}
