import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
//import DispenserItem from './dispenser-item';
import { SpotlightValue } from './spotlight-value';

export interface NegotiationsPanelProps {
  //dispenserStates: Readonly<Record<string, RomiCore.DispenserState>>;
  spotlight?: Readonly<SpotlightValue<string>>;
}

export default function NegotiationsPanel(props: NegotiationsPanelProps): JSX.Element {
  const { spotlight } = props;
  //const dispenserRefs = React.useRef<Record<string, HTMLElement | null>>({});
  //const [expanded, setExpanded] = React.useState<Record<string, boolean>>({});

  React.useEffect(() => {
    if (!spotlight) {
      return;
    }
    /*const ref = dispenserRefs.current[spotlight.value];
    if (!ref) {
      return;
    }
    setExpanded(prev => ({
      ...prev,
      [spotlight.value]: true,
    }));
    ref.scrollIntoView({ behavior: 'smooth' });
    */
  }, [spotlight]);


  return <React.Fragment></React.Fragment>;

}