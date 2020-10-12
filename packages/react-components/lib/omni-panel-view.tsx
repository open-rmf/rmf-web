import React from 'react';

export interface OmniPanelViewProps extends React.PropsWithChildren<unknown> {
  id: number;
}

export default function OmniPanelView(props: OmniPanelViewProps): JSX.Element {
  return <>{props.children}</>;
}
