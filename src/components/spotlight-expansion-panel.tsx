import { ExpansionPanelProps } from '@material-ui/core';
import React from 'react';

export interface SpotlightExpansionPanelProps extends ExpansionPanelProps {
  spotlight: any;
  children: React.ReactElement<ExpansionPanelProps & { key: any }>[];
}

export default function SpotlightExpansionPanel(props: SpotlightExpansionPanelProps): JSX.Element {
  const [expanded, setExpanded] = React.useState<Readonly<Record<string, boolean>>>({});
  const [currentSpotlight, setCurrentSpotlight] = React.useState<string | undefined>(undefined);

  const spotlight = props.spotlight;
  if (spotlight && spotlight !== currentSpotlight) {
    setExpanded(prev => {
      const newState = { ...prev };
      newState[spotlight] = true;
      return newState;
    });
    setCurrentSpotlight(props.spotlight);
  }

  function handleChange(key: any, newExpanded: boolean): void {
    setExpanded(prev => {
      const newState = { ...prev };
      newState[key] = newExpanded;
      return newState;
    });
  }

  for (const child of props.children) {
    child.props.onChange = (e, newExpanded) => {
      handleChange(child.props.key, newExpanded);
      child.props.onChange && child.props.onChange(e, newExpanded);
    };
    child.props.expanded = expanded[child.props.key] || child.props.expanded;
  }

  return <React.Fragment>{props.children}</React.Fragment>;
}
