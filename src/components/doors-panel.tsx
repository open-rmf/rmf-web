import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import DoorItem_, { DoorItemProps } from './door-item';
import { SpotlightValue } from './spotlight-value';

const debug = Debug('DoorsPanel');
const DoorItem = React.memo(DoorItem_);

export interface DoorsPanelProps {
  doors: readonly RomiCore.Door[];
  doorStates: Readonly<Record<string, RomiCore.DoorState>>;
  transport?: Readonly<RomiCore.Transport>;
  spotlight?: Readonly<SpotlightValue<string>>;
  onDoorClick?(door: RomiCore.Door): void;
}

function requestDoor(
  publisher: RomiCore.Publisher<RomiCore.DoorRequest>,
  requester_id: string,
  door: RomiCore.Door,
  mode: number,
): void {
  publisher.publish({
    door_name: door.name,
    requested_mode: { value: mode },
    requester_id: requester_id,
    request_time: RomiCore.toRosTime(new Date()),
  });
}

export const DoorsPanel = React.memo((props: DoorsPanelProps) => {
  debug('render');

  const { doors, doorStates, transport, spotlight, onDoorClick } = props;
  const doorRefs = React.useMemo(() => {
    const refs: Record<string, React.RefObject<HTMLElement>> = {};
    doors.map(door => (refs[door.name] = React.createRef()));
    return refs;
  }, [doors]);
  const [expanded, setExpanded] = React.useState<Record<string, boolean>>({});
  const doorRequestPub = React.useMemo(
    () => (transport ? transport.createPublisher(RomiCore.adapterDoorRequests) : null),
    [transport],
  );

  const onChange = React.useMemo(() => {
    return doors.reduce<Record<string, DoorItemProps['onChange']>>((prev, door) => {
      prev[door.name] = (_, newExpanded) => {
        setExpanded(prev => ({
          ...prev,
          [door.name]: newExpanded,
        }));
      };
      return prev;
    }, {});
  }, [doors]);

  const onClick = React.useMemo(() => {
    return doors.reduce<Record<string, DoorItemProps['onClick']>>((prev, door) => {
      prev[door.name] = () => {
        onDoorClick && onDoorClick(door);
      };
      return prev;
    }, {});
  }, [doors, onDoorClick]);

  const onOpenClick = React.useMemo(() => {
    return doors.reduce<Record<string, DoorItemProps['onOpenClick']>>((prev, door) => {
      prev[door.name] = () => {
        doorRequestPub &&
          transport &&
          requestDoor(doorRequestPub, transport.name, door, RomiCore.DoorMode.MODE_OPEN);
      };
      return prev;
    }, {});
  }, [doors, doorRequestPub, transport]);

  const onCloseClick = React.useMemo(() => {
    return doors.reduce<Record<string, DoorItemProps['onCloseClick']>>((prev, door) => {
      prev[door.name] = () => {
        doorRequestPub &&
          transport &&
          requestDoor(doorRequestPub, transport.name, door, RomiCore.DoorMode.MODE_CLOSED);
      };
      return prev;
    }, {});
  }, [doors, doorRequestPub, transport]);

  React.useEffect(() => {
    if (!spotlight) {
      return;
    }
    const ref = doorRefs[spotlight.value];
    if (!ref) {
      return;
    }
    setExpanded(prev => ({
      ...prev,
      [spotlight.value]: true,
    }));
    ref.current?.scrollIntoView({ behavior: 'smooth' });
  }, [spotlight, doorRefs]);

  const listItems = doors.map(door => {
    const doorState = doorStates[door.name];
    return (
      <DoorItem
        key={door.name}
        ref={doorRefs[door.name]}
        door={door}
        doorState={doorState}
        enableControls={Boolean(transport)}
        onOpenClick={onOpenClick[door.name]}
        onCloseClick={onCloseClick[door.name]}
        onClick={onClick[door.name]}
        expanded={Boolean(expanded[door.name])}
        onChange={onChange[door.name]}
      />
    );
  });

  return <React.Fragment>{listItems}</React.Fragment>;
});

export default DoorsPanel;
