import React from 'react';

export interface OmniPanelViewProps {
  value: number;
  index: number;
  children: React.ReactNode;
}

export default function OmniPanelView(props: OmniPanelViewProps): JSX.Element {
  const { value, index, children } = props;
  return <React.Fragment>{value === index && children}</React.Fragment>;
}
