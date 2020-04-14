import React from 'react';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import DispenserItem from './dispenser-item';
import { SpotlightValue } from './spotlight-expansion-panel';

export interface DispenserPanelProps {
  dispenserStates: Readonly<Record<string, RomiCore.DispenserState>>;
  spotlight?: Readonly<SpotlightValue<string>>;
}

export default function DispenserPanel(props: DispenserPanelProps): JSX.Element {
  const { spotlight } = props;
  const dispenserRefs = React.useRef<Record<string, HTMLElement | null>>({});
  const [expanded, setExpanded] = React.useState<Record<string, boolean>>({});

  React.useEffect(() => {
    if (!spotlight) {
      return;
    }
    const ref = dispenserRefs.current[spotlight.value];
    if (!ref) {
      return;
    }
    setExpanded(prev => ({
      ...prev,
      [spotlight.value]: true,
    }));
    ref.scrollIntoView({ behavior: 'smooth' });
  }, [spotlight]);

  const listItems = Object.keys(props.dispenserStates).map( (guid, index) => {
    const state = props.dispenserStates[guid];
    return (
      <DispenserItem
        key={state.guid}
        ref={ref => (dispenserRefs.current[state.guid] = ref)}
        dispenserState={state}
        expanded={Boolean(expanded[state.guid])}
        onChange={(_, newExpanded) =>
          setExpanded(prev => ({
            ...prev,
            [state.guid]: newExpanded,
          }))
        }
      />
    )
  });

  return <React.Fragment>{listItems}</React.Fragment>

}


