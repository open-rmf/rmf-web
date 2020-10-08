import { makeStyles, Slide } from '@material-ui/core';
import React from 'react';

const useStyles = makeStyles(() => ({
  exit: {
    position: 'absolute',
  },
}));

export interface OmniPanelViewProps extends React.PropsWithChildren<{}> {
  id: number;
}

export default function OmniPanelView(props: OmniPanelViewProps) {
  return <>{props.children}</>;
}
