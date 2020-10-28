import React from 'react';

export interface OmniPanelViewProps extends React.PropsWithChildren<unknown> {
  viewId: number;
}

export const OmniPanelView = (props: OmniPanelViewProps): JSX.Element => {
  return <>{props.children}</>;
};

export default OmniPanelView;
