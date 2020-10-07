import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import DispenserItem, { DispenserItemProps } from './dispenser-item';
import { SpotlightValue } from './spotlight-value';

const debug = Debug('OmniPanel:DispenserPanel');

export interface DispenserPanelProps {
  dispenserStates: Readonly<Record<string, RomiCore.DispenserState>>;
  spotlight?: Readonly<SpotlightValue<string>>;
}

export const DispenserPanel = React.memo((props: DispenserPanelProps) => {
  debug('render');

  const { dispenserStates, spotlight } = props;
  const dispenserRefs = React.useRef<Record<string, HTMLElement | null>>({});
  const [expanded, setExpanded] = React.useState<Record<string, boolean>>({});

  const storeRef = React.useCallback((ref: HTMLElement | null) => {
    if (!ref) {
      return;
    }
    const guid = ref.getAttribute('data-guid');
    if (!guid) {
      return;
    }
    dispenserRefs.current[guid] = ref;
  }, []);

  const onChange = React.useCallback<Required<DispenserItemProps>['onChange']>(
    (event, newExpanded) => {
      const guid = (event.currentTarget as HTMLElement).parentElement?.getAttribute('data-guid');
      if (!guid) {
        return;
      }
      setExpanded(prev => ({
        ...prev,
        [guid]: newExpanded,
      }));
    },
    [],
  );

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

  const listItems = Object.keys(dispenserStates).map((guid, i) => {
    const state = props.dispenserStates[guid];
    return (
      <DispenserItem
        key={state.guid}
        data-guid={state.guid}
        ref={storeRef}
        dispenserState={state}
        expanded={Boolean(expanded[state.guid])}
        onChange={onChange}
      />
    );
  });

  return <React.Fragment>{listItems}</React.Fragment>;
});

export default DispenserPanel;
