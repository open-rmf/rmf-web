import React from 'react';

import { LiftItem } from '../../components/lift-item/lift-item';
import { LiftsPanelProps } from '../../components/lift-item/lifts-panel';

export default function LiftButton(props: LiftsPanelProps) {
  const { lifts, liftStates } = props;
  const liftRefs = React.useRef<Record<string, HTMLElement | null>>({});

  return (
    <React.Fragment>
      {lifts.map(lift => {
        const liftState = liftStates[lift.name];
        return (
          <LiftItem
            key={lift.name}
            id={`Liftitem-${lift.name}`}
            lift={lift}
            ref={ref => (liftRefs.current[lift.name] = ref)}
            liftState={liftState}
          />
        );
      })}
    </React.Fragment>
  );
}
