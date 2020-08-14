import React from 'react';

import DoorItem from '../../components/door-item';
import { DoorsPanelProps } from '../../components/doors-panel';

export default function DoorButton(props: DoorsPanelProps) {
  const { doors, doorStates } = props;
  const doorRefs = React.useRef<Record<string, HTMLElement | null>>({});

  return (
    <React.Fragment>
      {doors.map(door => {
        const doorState = doorStates[door.name];
        return (
          <DoorItem
            key={door.name}
            ref={ref => (doorRefs.current[door.name] = ref)}
            door={door}
            doorState={doorState}
          />
        );
      })}
    </React.Fragment>
  );
}
