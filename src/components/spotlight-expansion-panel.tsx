import { ExpansionPanel, ExpansionPanelProps } from '@material-ui/core';
import React from 'react';

export interface SpotlightValue<T> {
  value: T;
}

export interface Props<T extends string | number> extends ExpansionPanelProps {
  index: T;

  /**
   * Use an object instead of a value so that it is possible to re-spotlight on an already
   * spotlighted panel.
   */
  spotlight?: SpotlightValue<T>;
}

/**
 * An extension to ExpansionPanel that adds a prop to open and focus on the panel.
 */
const SpotlightExpansionPanel = React.forwardRef(function<T extends string | number>(
  props: Props<T>,
  ref: React.Ref<unknown>,
): React.ReactElement {
  const innerRef = React.useRef<HTMLElement>(null);
  const [expanded, setExpanded] = React.useState(false);

  React.useEffect(() => {
    const spotlightValue = props.spotlight?.value;
    if (!spotlightValue || spotlightValue !== props.index) {
      return;
    }
    setExpanded(true);
    innerRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [props.spotlight, props.index, ref]);

  React.useEffect(() => {
    if (typeof ref === 'function') {
      ref(innerRef.current);
    } else if (ref !== null) {
      (ref.current as any) = innerRef.current; // force mutation :monkaS:
    }
  });

  function handleChange(e: React.ChangeEvent<{}>, newExpanded: boolean) {
    setExpanded(newExpanded);
    props.onChange && props.onChange(e, newExpanded);
  }

  return (
    <ExpansionPanel
      ref={innerRef}
      expanded={Boolean(expanded || props.expanded)}
      onChange={handleChange}
    >
      {props.children}
    </ExpansionPanel>
  );
});

export default SpotlightExpansionPanel;
