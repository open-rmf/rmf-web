import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import { makeCallbackArrayCallback } from '../util/react-helpers';
import DoorItem_, { DoorItemProps } from './door-item';
import { SpotlightValue } from './spotlight-value';

const debug = Debug('DoorsPanel');
const DoorItem = React.memo(DoorItem_);

export interface DoorsPanelProps {
  doors: RomiCore.Door[];
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
  const [expanded, setExpanded] = React.useState<Record<string, boolean>>({});
  const doorRequestPub = React.useMemo(
    () => (transport ? transport.createPublisher(RomiCore.adapterDoorRequests) : null),
    [transport],
  );

  const doorRefs = React.useMemo(() => {
    const refs: Record<string, React.RefObject<HTMLElement>> = {};
    doors.map(door => (refs[door.name] = React.createRef()));
    return refs;
  }, [doors]);

  const onChange = React.useMemo(
    makeCallbackArrayCallback<Required<DoorItemProps>['onChange'], RomiCore.Door>(
      doors,
      (door, _, newExpanded) =>
        setExpanded(prev => ({
          ...prev,
          [door.name]: newExpanded,
        })),
    ),
    [doors],
  );

  const onClick = React.useMemo(
    makeCallbackArrayCallback<Required<DoorItemProps>['onClick'], RomiCore.Door>(
      doors,
      door => onDoorClick && onDoorClick(door),
    ),
    [doors, onDoorClick],
  );

  const onOpenClick = React.useMemo(
    makeCallbackArrayCallback<Required<DoorItemProps>['onOpenClick'], RomiCore.Door>(
      doors,
      door =>
        doorRequestPub &&
        transport &&
        requestDoor(doorRequestPub, transport.name, door, RomiCore.DoorMode.MODE_OPEN),
    ),
    [doors, onDoorClick],
  );

  const onCloseClick = React.useMemo(
    makeCallbackArrayCallback<Required<DoorItemProps>['onCloseClick'], RomiCore.Door>(
      doors,
      door =>
        doorRequestPub &&
        transport &&
        requestDoor(doorRequestPub, transport.name, door, RomiCore.DoorMode.MODE_OPEN),
    ),
    [doors, onDoorClick],
  );

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

  const listItems = doors.map((door, i) => {
    const doorState = doorStates[door.name];
    return (
      <DoorItem
        key={door.name}
        ref={doorRefs[door.name]}
        door={door}
        doorState={doorState}
        enableControls={Boolean(transport)}
        onOpenClick={onOpenClick[i]}
        onCloseClick={onCloseClick[i]}
        onClick={onClick[i]}
        expanded={Boolean(expanded[door.name])}
        onChange={onChange[i]}
      />
    );
  });

  return <React.Fragment>{listItems}</React.Fragment>;
});

export default DoorsPanel;
