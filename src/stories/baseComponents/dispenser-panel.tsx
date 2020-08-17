import React from 'react';

import { DispenserItem } from '../../components/dispenser-item';
import { DispenserPanelProps } from '../../components/dispensers-panel';

export default function DispenserButton(props: DispenserPanelProps) {
  const { dispenserStates } = props;
  const dispenserRefs = React.useRef<Record<string, HTMLElement | null>>({});

  return (
    <React.Fragment>
      {Object.keys(dispenserStates).map(guid => {
        const state = dispenserStates[guid];
        return (
          <DispenserItem
            key={state.guid}
            ref={ref => (dispenserRefs.current[state.guid] = ref)}
            dispenserState={state}
          />
        );
      })}
    </React.Fragment>
  );
}
