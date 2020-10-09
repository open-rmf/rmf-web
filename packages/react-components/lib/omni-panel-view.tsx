import React from 'react';

export interface OmniPanelViewProps extends React.PropsWithChildren<{}> {
  id: number;
}

export default function OmniPanelView(props: OmniPanelViewProps) {
  return <>{props.children}</>;
}
