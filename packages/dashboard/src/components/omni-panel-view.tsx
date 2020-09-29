import React from 'react';

export interface OmniPanelViewProps extends React.PropsWithChildren<{}> {
  id: number;
}

export default function OmniPanelView(props: OmniPanelViewProps): React.ReactElement {
  return <React.Fragment>{props.children}</React.Fragment>;
}
